#!/usr/bin/env python3
"""
=============================================================================
CAVE EXPLORER AI · TACTICAL DESKTOP MISSION CONTROL [MR. ROBOT FSOCIETY EDITION]
=============================================================================
Fully converted Python Desktop Application embedding the exact 100% UI/UX of
the Tactical Dashboards while preserving all backend pipeline logic from
arduinoclient1.py:
  1. Receives JPEG fragments from ESP32-CAM over UDP (Port 4210)
  2. Runs corridor obstacle detection & yellow surface mask
  3. Sends rate-limited GO/STOP decision packets to Arduino UNO R4 over UDP (Port 4212)
  4. Controls ESP32 LED flashlight over UDP (Port 4211)
  5. Hosts high-performance local MJPEG streams and telemetry endpoints
  6. Renders the exact 100% same Mr. Robot themed UI/UX in a native desktop window
=============================================================================
"""

import sys
import os
import time
import socket
import threading
import json
import numpy as np
import cv2
from flask import Flask, Response, jsonify, request, send_from_directory
import webview

# =====================================================================
# CONFIGURATION — STRICTLY ALIGNED WITH arduinoclient1.py
# =====================================================================
ESP32_IP     = "10.151.173.181"     # ESP32-CAM IP (from arduinoclientnew.py)
ARDUINO_IP   = "10.151.173.226"     # Arduino UNO R4 WiFi IP

LISTEN_PORT  = 4210                 # ESP32 sends JPEG fragments here
CTRL_PORT    = 4211                 # ESP32 LED control (LED_ON / LED_OFF / LED_TOGGLE)
ARDUINO_PORT = 4212                 # Arduino vision decision sink
VISION_MAGIC = 0xC8                 # Must match Arduino sketch (200 decimal)
SEND_INTERVAL= 0.033                # Max 30 Hz to Arduino

# Vision Packet Command Codes (strictly matching car_firmware.ino)
VISION_CMD_HAZARD_STOP = 0   # AI sees an obstacle -> temporary brake
VISION_CMD_HAZARD_GO   = 1   # Corridor clear      -> resume
VISION_CMD_OP_STOP     = 2   # STOP button         -> latch motors OFF (DISARM)
VISION_CMD_OP_START    = 3   # START button        -> latch motors ON (ARM)

# Reassembly parameters
MAX_PAYLOAD  = 1400
HDR_SIZE     = 4
STATS_EVERY  = 2.0

# Computer Vision HSV Corridor Yellow Range
config = {
    "lower_yellow": [18, 60, 60],
    "upper_yellow": [45, 255, 255],
    "roi_top_frac": 0.40,
    "roi_x1_frac": 0.15,
    "roi_x2_frac": 0.85,
    "min_obstacle_area_frac": 0.008,
    "stop_area_frac": 0.030,
    "esp32_ip": ESP32_IP,
    "arduino_ip": ARDUINO_IP
}

# =====================================================================
# GLOBAL SHARED STATE
# =====================================================================
state = {
    "latest_raw": None,
    "latest_annotated": None,
    "latest_mask": None,
    "last_fid": -1,
    "fps": 0.0,
    "decision": 1,
    "reason": "Corridor Clear",
    "obstacle_ratio": 0.0,
    "boxes_count": 0,
    "flash_state": False,
    "status_led": False,
    "last_packet_time": 0,
    "last_udp_time": 0,
    "frames_received": 0,
    "tx_packets": 0,
    "arduino_seq": 0,
    "drive_armed": False,
    "running": True
}

lock = threading.Lock()

# Determine directory paths
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
app = Flask(__name__, static_folder=BASE_DIR)

@app.after_request
def add_cors_headers(response):
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type,Authorization'
    response.headers['Access-Control-Allow-Methods'] = 'GET,POST,OPTIONS'
    return response

# =====================================================================
# SOCKETS (ARDUINO DECISIONS & ESP32 CONTROL)
# =====================================================================
ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
arduino_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

arduino_seq = 0
last_send_time = 0
sent_packets = 0
last_sent_decision = None
first_send_done = False

def send_esp_cmd(cmd: str) -> bool:
    try:
        ctrl_sock.sendto(cmd.encode(), (config["esp32_ip"], CTRL_PORT))
        print(f"[CTRL] Sent '{cmd}' -> {config['esp32_ip']}:{CTRL_PORT}")
        return True
    except Exception as e:
        print(f"[CTRL ERR] {e}")
        return False

def send_operator_command(cmd_code: int):
    """Send immediate operator START/STOP command burst (3 packets) to Arduino UNO R4."""
    global arduino_seq, sent_packets
    with lock:
        arduino_seq = (arduino_seq + 1) & 0xFF
        seq = arduino_seq
        if cmd_code == VISION_CMD_OP_START:
            state["drive_armed"] = True
        elif cmd_code == VISION_CMD_OP_STOP:
            state["drive_armed"] = False

    # Packet format: [MAGIC, CMD_CODE, CONFIDENCE, SEQ]
    pkt = bytes([VISION_MAGIC, cmd_code & 0xFF, 100, seq])
    for _ in range(3):
        try:
            arduino_sock.sendto(pkt, (config["arduino_ip"], ARDUINO_PORT))
            sent_packets += 1
        except Exception as e:
            print(f"[OP ERR] {e}")
        time.sleep(0.004)

    with lock:
        state["tx_packets"] = sent_packets
        state["arduino_seq"] = seq

    name = "START (ARM MOTORS)" if cmd_code == VISION_CMD_OP_START else "STOP (DISARM / BRAKE)"
    print(f"[OPERATOR] >>> {name} (code={cmd_code}, seq={seq}) -> {config['arduino_ip']}:{ARDUINO_PORT} <<<")

def send_vision_decision(decision: int, confidence: int = 100):
    """Send binary decision packet to Arduino UNO R4 WiFi over UDP (from arduinoclient1.py)."""
    global arduino_seq, last_send_time, sent_packets, last_sent_decision, first_send_done
    now = time.time()
    if now - last_send_time < SEND_INTERVAL:
        return
    last_send_time = now
    arduino_seq = (arduino_seq + 1) & 0xFF
    pkt = bytes([VISION_MAGIC, int(decision) & 0xFF, int(confidence) & 0xFF, arduino_seq])
    try:
        arduino_sock.sendto(pkt, (config["arduino_ip"], ARDUINO_PORT))
        sent_packets += 1
        with lock:
            state["tx_packets"] = sent_packets
            state["arduino_seq"] = arduino_seq

        if not first_send_done:
            first_send_done = True
            print(f"[TX] First vision packet sent -> {config['arduino_ip']}:{ARDUINO_PORT}")
        if decision != last_sent_decision:
            tag = "STOP" if decision == 0 else "GO  "
            print(f"[TX] {tag} -> {config['arduino_ip']}:{ARDUINO_PORT} (seq={arduino_seq})")
            last_sent_decision = decision
    except Exception as e:
        print(f"[TX ERR] Arduino send failed: {e}")

# =====================================================================
# COMPUTER VISION ALGORITHM (EXACTLY FROM arduinoclient1.py)
# =====================================================================
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
    obstacle = cv2.bitwise_and(obstacle, roi_mask)

    # Morphological noise cleanup
    obstacle = cv2.morphologyEx(obstacle, cv2.MORPH_OPEN, KERNEL)
    obstacle = cv2.morphologyEx(obstacle, cv2.MORPH_CLOSE, KERNEL)

    # Contours
    contours, _ = cv2.findContours(obstacle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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

    ratio = total_obs_area / roi_area if roi_area > 0 else 0
    decision = 0 if ratio >= config["stop_area_frac"] else 1
    reason = f"Obstacle {ratio*100:.1f}% of corridor" if decision == 0 else "Corridor Clear"

    # Annotate frame with tactical HUD markings
    annotated = bgr.copy()
    cv2.rectangle(annotated, (x0, y0), (x1, H), (0, 255, 65) if decision == 1 else (0, 0, 229), 2)

    for (bx0, by0, bx1_b, by1_b, area) in boxes:
        cv2.rectangle(annotated, (bx0, by0), (bx1_b, by1_b), (0, 0, 255), 2)
        cv2.putText(annotated, f"{area/roi_area*100:.1f}%", (bx0, max(14, by0 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

    # Top tactical decision badge
    col = (0, 180, 0) if decision == 1 else (0, 0, 220)
    txt = f"GO   | {reason}" if decision == 1 else f"STOP | {reason}"
    cv2.rectangle(annotated, (0, 0), (W, 22), col, -1)
    cv2.putText(annotated, txt, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

    # Bottom hardware telemetry stamp
    tx_txt = f"TX -> {config['arduino_ip']}:{ARDUINO_PORT}  pkts={sent_packets}"
    cv2.putText(annotated, tx_txt, (6, H - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1, cv2.LINE_AA)

    return decision, reason, ratio, len(boxes), annotated, obstacle

# =====================================================================
# UDP VIDEO RECEIVER THREAD (EXACTLY FROM arduinoclient1.py)
# =====================================================================
def udp_receiver_loop():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
    try:
        sock.bind(("0.0.0.0", LISTEN_PORT))
    except Exception as e:
        print(f"[VIDEO ERR] Cannot bind to 0.0.0.0:{LISTEN_PORT}: {e}")
        return

    sock.settimeout(0.3)
    print(f"[VIDEO] Receiver listening on UDP :{LISTEN_PORT}")

    frames = {}
    last_displayed_fid = -1
    frames_shown = 0
    t0 = time.time()

    def purge_stale(cur_fid):
        for k in [k for k in frames if k < cur_fid - 1]:
            del frames[k]

    # ESP32-CAM HTTP Stream & Handshake Worker
    def esp32_camera_worker():
        http_cap = None
        current_http_url = None
        last_handshake = 0

        def check_tcp(host, port, timeout=0.15):
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(timeout)
                res = s.connect_ex((host, port))
                s.close()
                return res == 0
            except Exception:
                return False

        fb_frames = 0
        fb_t0 = time.time()

        while state["running"]:
            src = config.get("camera_source", "auto")
            udp_active = (time.time() - state.get("last_udp_time", 0)) < 2.0
            esp_ip = config.get("esp32_ip", ESP32_IP)

            # Periodically send handshake packet to ESP32 on port 4211 to register laptop IP
            now = time.time()
            if now - last_handshake > 2.0 and not udp_active:
                last_handshake = now
                try:
                    ctrl_sock.sendto(b"START", (esp_ip, CTRL_PORT))
                    ctrl_sock.sendto(b"PING", (esp_ip, CTRL_PORT))
                except Exception:
                    pass

            # If UDP is actively streaming from ESP32, release HTTP stream
            if udp_active:
                if http_cap is not None:
                    http_cap.release()
                    http_cap = None
                time.sleep(0.1)
                continue

            frame = None

            # Try ESP32-CAM HTTP streams (:81/stream, :80/stream, or custom URL)
            stream_url = config.get("http_stream_url") or f"http://{esp_ip}:81/stream"

            if src in ("http", "auto"):
                host = esp_ip.split(":")[0] if ":" in esp_ip else esp_ip
                port = int(esp_ip.split(":")[1]) if ":" in esp_ip else 81

                if check_tcp(host, port) or check_tcp(host, 80):
                    if http_cap is None or current_http_url != stream_url:
                        if http_cap is not None:
                            http_cap.release()
                        http_cap = cv2.VideoCapture(stream_url)
                        current_http_url = stream_url

                    if http_cap.isOpened():
                        ret, f = http_cap.read()
                        if ret and f is not None:
                            frame = f

            if frame is not None:
                decision, reason, ratio, n_boxes, annotated, mask = analyze_frame(frame)
                send_vision_decision(decision, confidence=100)

                with lock:
                    state["latest_raw"] = frame
                    state["latest_annotated"] = annotated
                    state["latest_mask"] = mask
                    state["decision"] = decision
                    state["reason"] = reason
                    state["obstacle_ratio"] = ratio
                    state["boxes_count"] = n_boxes
                    state["last_packet_time"] = time.time()
                    state["frames_received"] += 1

                fb_frames += 1
                now = time.time()
                if now - fb_t0 >= 1.0:
                    with lock:
                        state["fps"] = round(fb_frames / (now - fb_t0), 1)
                    fb_frames = 0
                    fb_t0 = now

                time.sleep(0.033)
            else:
                time.sleep(0.1)

        if http_cap: http_cap.release()

    camera_thread = threading.Thread(target=esp32_camera_worker, daemon=True)
    camera_thread.start()

    while state["running"]:
        try:
            data, _ = sock.recvfrom(MAX_PAYLOAD + HDR_SIZE + 64)
        except socket.timeout:
            continue
        except Exception:
            time.sleep(0.01)
            continue

        if len(data) < HDR_SIZE + 1:
            continue

        fid   = (data[0] << 8) | data[1]
        pidx  =  data[2]
        total =  data[3]
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

                    # Send vision decision to Arduino UNO R4 WiFi
                    send_vision_decision(decision, confidence=100)

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
                        state["last_udp_time"] = time.time()
                        state["frames_received"] += 1

                    last_displayed_fid = fid
                    frames_shown += 1

            del frames[fid]

        now = time.time()
        if now - t0 >= STATS_EVERY:
            fps = frames_shown / (now - t0)
            with lock:
                state["fps"] = round(fps, 1)
            frames_shown = 0
            t0 = now

    sock.close()

# =====================================================================
# EMBEDDED FLASK ROUTES (FOR EXACT 100% SAME UI/UX)
# =====================================================================
def generate_mjpeg(feed_type="annotated"):
    while state["running"]:
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
            esp_ip = config.get("esp32_ip", ESP32_IP)

        if img is not None and not is_stale:
            _, buffer = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            frame_bytes = buffer.tobytes()
        else:
            blank = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(blank, "ESP32-CAM MODULE WAITING", (35, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(blank, f"Target IP: {esp_ip}", (35, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 65), 1, cv2.LINE_AA)
            cv2.putText(blank, "Listening on UDP :4210", (35, 145),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1, cv2.LINE_AA)
            cv2.putText(blank, "Awaiting ESP32 module packets...", (35, 170),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (120, 120, 120), 1, cv2.LINE_AA)
            _, blank_jpg = cv2.imencode('.jpg', blank)
            frame_bytes = blank_jpg.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.033)

@app.route('/video_feed')
def video_feed():
    feed_type = request.args.get('type', 'annotated')
    return Response(generate_mjpeg(feed_type), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/raw_feed')
def raw_feed():
    return Response(generate_mjpeg("raw"), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/mask_feed')
def mask_feed():
    return Response(generate_mjpeg("mask"), mimetype='multipart/x-mixed-replace; boundary=frame')

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
            "drive_armed": state.get("drive_armed", False),
            "esp32_ip": config["esp32_ip"],
            "arduino_ip": config["arduino_ip"],
            "tx_packets": state["tx_packets"],
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

@app.route('/api/robot_command', methods=['POST'])
def api_robot_command():
    data = request.get_json(force=True) or {}
    cmd = str(data.get("command", "")).strip().lower()
    char = str(data.get("char", "")).strip().lower()
    print(f"[CMD] Robot command received: '{cmd}' (char='{char}')")

    if cmd in ("start", "forward", "fwd", "f", "arm", "run") or char in ("f", "w"):
        send_operator_command(VISION_CMD_OP_START)
        send_vision_decision(VISION_CMD_HAZARD_GO, 100)
    elif cmd in ("stop", "s", "brake", "disarm", "halt") or char in ("s", " "):
        send_operator_command(VISION_CMD_OP_STOP)
        send_vision_decision(VISION_CMD_HAZARD_STOP, 100)
    elif cmd in ("hazard_stop",):
        send_vision_decision(VISION_CMD_HAZARD_STOP, 100)
    elif cmd in ("hazard_go",):
        send_vision_decision(VISION_CMD_HAZARD_GO, 100)

    with lock:
        armed = state.get("drive_armed", False)

    return jsonify({"success": True, "command": cmd, "char": char, "drive_armed": armed})

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
def serve_cam_dashboard():
    return send_from_directory(BASE_DIR, "cam-dashboard.html")

@app.route('/cave-ai.html')
def serve_cave_dashboard():
    return send_from_directory(BASE_DIR, "cave-ai.html")

@app.route('/<path:filename>')
def serve_static(filename):
    return send_from_directory(BASE_DIR, filename)

# =====================================================================
# APPLICATION ENTRYPOINT & NATIVE DESKTOP WINDOW LAUNCHER
# =====================================================================
def run_flask():
    app.run(host="127.0.0.1", port=5000, threaded=True, debug=False)

def on_window_closed():
    state["running"] = False
    print("[DESKTOP] Window closed. Shutting down pipeline.")

def main():
    print("=" * 70)
    print("  SENTINEL-MINE · THE CAVE EXPLORER ROBOT")
    print("  TACTICAL MISSION CONTROL")
    print("=" * 70)
    print(f"  ESP32-CAM Source    : {config['esp32_ip']}:{LISTEN_PORT}")
    print(f"  Arduino Decision Sink: {config['arduino_ip']}:{ARDUINO_PORT}")
    print(f"  Local Tactical Server: http://127.0.0.1:5000")
    print("=" * 70)

    # 1. Start UDP pipeline thread
    t_udp = threading.Thread(target=udp_receiver_loop, daemon=True)
    t_udp.start()

    # 2. Start Local Flask server thread
    t_flask = threading.Thread(target=run_flask, daemon=True)
    t_flask.start()

    time.sleep(0.5)

    # 3. Create native desktop application window with pywebview
    window = webview.create_window(
        title="Sentinel-Mine the Cave Explorer Robot",
        url="http://127.0.0.1:5000/cam-dashboard.html",
        width=1440,
        height=900,
        min_size=(1024, 720),
        background_color="#050505",
        text_select=False,
        zoomable=True
    )
    window.events.closed += on_window_closed

    # Start desktop GUI loop
    webview.start(debug=False)
    state["running"] = False
    sys.exit(0)

if __name__ == '__main__':
    main()
