import socket
import time
import threading
import json
import numpy as np
import cv2
from flask import Flask, Response, jsonify, request, send_from_directory
import os

# ---------------- CONFIGURATION ----------------
ESP32_IP = "10.160.191.130"
LISTEN_PORT = 4210
CTRL_PORT = 4211
MAX_PAYLOAD = 1400
HDR_SIZE = 4

# HSV Floor Surface Range (Yellow ground)
config = {
    "lower_yellow": [18, 60, 60],
    "upper_yellow": [45, 255, 255],
    "roi_top_frac": 0.40,
    "roi_x1_frac": 0.15,
    "roi_x2_frac": 0.85,
    "min_obstacle_area_frac": 0.008,
    "stop_area_frac": 0.030,
    "esp32_ip": ESP32_IP
}

# ---------------- SHARED STATE ----------------
state = {
    "latest_raw": None,
    "latest_annotated": None,
    "latest_mask": None,
    "last_fid": -1,
    "fps": 0.0,
    "decision": 1,
    "reason": "Waiting for stream...",
    "obstacle_ratio": 0.0,
    "boxes_count": 0,
    "flash_state": False,
    "status_led": False,
    "last_packet_time": 0,
    "frames_received": 0
}

lock = threading.Lock()
app = Flask(__name__, static_folder="../")

# Add CORS headers to all responses
@app.after_request
def add_cors_headers(response):
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type,Authorization'
    response.headers['Access-Control-Allow-Methods'] = 'GET,POST,OPTIONS'
    return response

# ---------------- UDP CONTROL SOCKET ----------------
ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_esp_cmd(cmd: str):
    try:
        ctrl_sock.sendto(cmd.encode(), (config["esp32_ip"], CTRL_PORT))
        print(f"[CTRL SENT] {cmd} -> {config['esp32_ip']}:{CTRL_PORT}")
        return True
    except Exception as e:
        print("[CTRL ERR]", e)
        return False

# ---------------- COMPUTER VISION PIPELINE ----------------
KERNEL = np.ones((5, 5), np.uint8)

def analyze_frame(bgr):
    H, W = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    
    lower = np.array(config["lower_yellow"])
    upper = np.array(config["upper_yellow"])
    
    yellow = cv2.inRange(hsv, lower, upper)
    obstacle = cv2.bitwise_not(yellow)

    # Restrict to forward corridor ROI
    y0 = int(H * config["roi_top_frac"])
    x0 = int(W * config["roi_x1_frac"])
    x1 = int(W * config["roi_x2_frac"])
    
    roi_mask = np.zeros_like(obstacle)
    roi_mask[y0:H, x0:x1] = 255
    obstacle_roi = cv2.bitwise_and(obstacle, roi_mask)

    # Morphological noise cleanup
    cleaned = cv2.morphologyEx(obstacle_roi, cv2.MORPH_OPEN, KERNEL)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, KERNEL)

    # Contours
    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    roi_area = max(1, (H - y0) * (x1 - x0))
    min_area = config["min_obstacle_area_frac"] * roi_area

    boxes = []
    total_obs_area = 0
    for c in contours:
        a = cv2.contourArea(c)
        if a < min_area:
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        boxes.append((bx, by, bx + bw, by + bh, a))
        total_obs_area += a

    ratio = total_obs_area / roi_area
    decision = 0 if ratio >= config["stop_area_frac"] else 1
    reason = f"Obstacle {ratio*100:.1f}% of corridor" if decision == 0 else "Corridor Clear"

    # Annotate frame
    annotated = bgr.copy()
    
    # Draw corridor ROI with translucent fill
    roi_overlay = annotated.copy()
    cv2.rectangle(roi_overlay, (x0, y0), (x1, H), (60, 200, 60) if decision == 1 else (60, 60, 220), -1)
    cv2.addWeighted(roi_overlay, 0.12, annotated, 0.88, 0, annotated)
    cv2.rectangle(annotated, (x0, y0), (x1, H), (0, 255, 120) if decision == 1 else (0, 70, 255), 2)

    # Draw obstacle bounding boxes
    for (bx0, by0, bx1_b, by1_b, area) in boxes:
        cv2.rectangle(annotated, (bx0, by0), (bx1_b, by1_b), (0, 0, 255), 2)
        cv2.putText(annotated, f"{area/roi_area*100:.1f}%", (bx0, max(16, by0 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1, cv2.LINE_AA)

    # Top Status HUD Banner
    col = (20, 160, 20) if decision == 1 else (20, 20, 200)
    cv2.rectangle(annotated, (0, 0), (W, 26), col, -1)
    banner_txt = f"{'CLEAR (GO)' if decision == 1 else 'OBSTACLE (STOP)'}  |  {reason}"
    cv2.putText(annotated, banner_txt, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)

    return decision, reason, ratio, len(boxes), annotated, cleaned

# ---------------- UDP RX WORKER THREAD ----------------
def udp_receiver():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
    except Exception:
        pass
    sock.bind(("0.0.0.0", LISTEN_PORT))
    sock.settimeout(0.5)
    print(f"[*] ESP32-CAM UDP receiver listening on 0.0.0.0:{LISTEN_PORT}")

    frames = {}
    last_displayed_fid = -1
    frames_shown = 0
    t0 = time.time()

    def purge_stale(cur_fid):
        for k in [k for k in frames if k < cur_fid - 1]:
            del frames[k]

    while True:
        try:
            data, _ = sock.recvfrom(MAX_PAYLOAD + HDR_SIZE + 64)
        except socket.timeout:
            continue
        except Exception as e:
            time.sleep(0.01)
            continue

        if len(data) < HDR_SIZE + 1:
            continue

        fid = (data[0] << 8) | data[1]
        pidx = data[2]
        total = data[3]
        payload = data[HDR_SIZE:]

        purge_stale(fid)
        bucket = frames.setdefault(fid, {})
        bucket[pidx] = payload

        if len(bucket) == total:
            if fid > last_displayed_fid:
                jpg = b"".join(bucket[i] for i in range(total))
                arr = np.frombuffer(jpg, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    decision, reason, ratio, n_boxes, annotated, mask = analyze_frame(img)
                    with lock:
                        state["latest_raw"] = img
                        state["latest_annotated"] = annotated
                        state["latest_mask"] = mask
                        state["last_fid"] = fid
                        state["decision"] = decision
                        state["reason"] = reason
                        state["obstacle_ratio"] = ratio
                        state["boxes_count"] = n_boxes
                        state["last_packet_time"] = time.time()
                        state["frames_received"] += 1

                    last_displayed_fid = fid
                    frames_shown += 1

            del frames[fid]

        now = time.time()
        if now - t0 >= 1.0:
            with lock:
                state["fps"] = round(frames_shown / (now - t0), 1)
            frames_shown = 0
            t0 = now

# ---------------- FLASK ROUTES & MJPEG STREAMS ----------------
def generate_mjpeg(feed_type="annotated"):
    blank = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.putText(blank, "Awaiting ESP32-CAM Stream...", (25, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (140, 140, 140), 1, cv2.LINE_AA)
    _, blank_jpg = cv2.imencode('.jpg', blank)
    blank_bytes = blank_jpg.tobytes()

    while True:
        with lock:
            img = None
            if feed_type == "annotated":
                img = state["latest_annotated"]
            elif feed_type == "raw":
                img = state["latest_raw"]
            elif feed_type == "mask":
                mask = state["latest_mask"]
                if mask is not None:
                    img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            is_stale = (time.time() - state["last_packet_time"]) > 2.5

        if img is not None and not is_stale:
            _, buffer = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            frame_bytes = buffer.tobytes()
        else:
            frame_bytes = blank_bytes

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.04)

@app.route('/video_feed')
def video_feed():
    return Response(generate_mjpeg("annotated"),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/raw_feed')
def raw_feed():
    return Response(generate_mjpeg("raw"),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/mask_feed')
def mask_feed():
    return Response(generate_mjpeg("mask"),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/stats')
def api_stats():
    with lock:
        is_live = (time.time() - state["last_packet_time"]) < 2.5
        return jsonify({
            "is_live": is_live,
            "fps": state["fps"] if is_live else 0.0,
            "decision": state["decision"],
            "reason": state["reason"],
            "obstacle_ratio": round(state["obstacle_ratio"] * 100, 1),
            "boxes_count": state["boxes_count"],
            "frame_id": state["last_fid"],
            "frames_total": state["frames_received"],
            "flash_on": state["flash_state"],
            "status_on": state["status_led"],
            "esp32_ip": config["esp32_ip"],
            "config": config
        })

@app.route('/api/control', methods=['POST'])
def api_control():
    data = request.get_json(force=True) or {}
    cmd = data.get("cmd", "")
    if not cmd:
        return jsonify({"success": False, "error": "No command provided"}), 400

    if cmd == "LED_ON":
        state["flash_state"] = True
    elif cmd == "LED_OFF":
        state["flash_state"] = False
    elif cmd == "LED_TOGGLE":
        state["flash_state"] = not state["flash_state"]
    elif cmd == "STATUS_ON":
        state["status_led"] = True
    elif cmd == "STATUS_OFF":
        state["status_led"] = False

    ok = send_esp_cmd(cmd)
    return jsonify({"success": ok, "cmd": cmd, "flash_on": state["flash_state"], "status_on": state["status_led"]})

@app.route('/api/config', methods=['GET', 'POST'])
def api_config():
    if request.method == 'POST':
        data = request.get_json(force=True) or {}
        for k in config:
            if k in data:
                config[k] = data[k]
        return jsonify({"success": True, "config": config})
    return jsonify(config)

@app.route('/')
@app.route('/cam-dashboard.html')
def serve_dashboard():
    dashboard_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    return send_from_directory(dashboard_path, "cam-dashboard.html")

@app.route('/cave-ai.html')
def serve_cave_ai():
    dashboard_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    return send_from_directory(dashboard_path, "cave-ai.html")

if __name__ == '__main__':
    t = threading.Thread(target=udp_receiver, daemon=True)
    t.start()
    print("[*] Starting Flask Vision Server on http://0.0.0.0:5000")
    app.run(host="0.0.0.0", port=5000, threaded=True, debug=False)
