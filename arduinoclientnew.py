# =====================================================================
# ESP32-CAM → Laptop → Arduino pipeline
#   1. Receives JPEG frames from ESP32-CAM over UDP (port 4210)
#   2. Runs obstacle detection (color mask on yellow surface)
#   3. Sends GO/STOP decision to Arduino UNO R4 WiFi over UDP (port 4212)
#   4. Shows annotated frame in a cv2 window
#   5. Optional: toggles ESP32 white LED over UDP (port 4211)
# =====================================================================

import socket
import sys
import time
import numpy as np
import cv2

# =====================================================================
# CONFIG — FILL THESE IN
# =====================================================================
ESP32_IP    = "10.151.173.181"     # <-- ESP32-CAM IP (from ESP32 Serial Monitor)
ARDUINO_IP  = "10.151.173.226"     # <-- Arduino UNO R4 WiFi IP (from Arduino Serial Monitor)

LISTEN_PORT = 4210                  # ESP32 sends JPEG fragments here
CTRL_PORT   = 4211                  # ESP32 LED control (LED_ON/LED_OFF)
ARDUINO_PORT = 4212                 # Arduino vision decisions
VISION_MAGIC = 0xC8                 # must match Arduino sketch
SEND_INTERVAL = 0.033               # max 30 Hz to Arduino

# Reassembly
MAX_PAYLOAD = 1400
HDR_SIZE    = 4
STATS_EVERY = 2.0

# ---- Yellow surface HSV range ----
LOWER_YELLOW = np.array([18, 60, 60])
UPPER_YELLOW = np.array([45, 255, 255])

# ---- Forward corridor ROI ----
ROI_TOP_FRAC = 0.40
ROI_X1_FRAC  = 0.15
ROI_X2_FRAC  = 0.85

# ---- Decision thresholds ----
MIN_OBSTACLE_AREA_FRAC = 0.008
STOP_AREA_FRAC         = 0.030

# ---- Optional YOLO secondary gate ----
USE_YOLO = False
YOLO_PERSON_GATE = False

# =====================================================================
# STARTUP VALIDATION — refuse to run with bad config
# =====================================================================
def _is_valid_ip(s):
    if s in ("", "0.0.0.0", "localhost", "127.0.0.1"):
        return False
    parts = s.split(".")
    if len(parts) != 4:
        return False
    try:
        return all(0 <= int(p) <= 255 for p in parts)
    except ValueError:
        return False

if not _is_valid_ip(ESP32_IP):
    print("[FATAL] ESP32_IP is not set correctly:", ESP32_IP)
    print("        Get it from the ESP32-CAM Serial Monitor at boot:")
    print("          >>> ESP32 IP: x.x.x.x")
    sys.exit(1)

if not _is_valid_ip(ARDUINO_IP):
    print("[FATAL] ARDUINO_IP is not set correctly:", ARDUINO_IP)
    print("        Get it from the Arduino Serial Monitor at boot:")
    print("          Arduino IP: x.x.x.x")
    print("        Then edit this script's ARDUINO_IP variable and rerun.")
    sys.exit(1)

print("=" * 60)
print("  Pipeline configuration")
print("=" * 60)
print(f"  ESP32-CAM (video source) : {ESP32_IP}:{LISTEN_PORT}")
print(f"  Arduino   (decision sink): {ARDUINO_IP}:{ARDUINO_PORT}")
print("=" * 60)

# =====================================================================
# SOCKETS
# =====================================================================
# Video receiver
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
sock.bind(("0.0.0.0", LISTEN_PORT))
sock.settimeout(0.3)
print(f"[VIDEO] Listening on UDP :{LISTEN_PORT}")

# LED control sender
ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def send_led_cmd(cmd: str):
    try:
        ctrl_sock.sendto(cmd.encode(), (ESP32_IP, CTRL_PORT))
    except Exception as e:
        print("LED ctrl send failed:", e)

# Arduino decision sender
arduino_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
arduino_seq = 0
last_send_time = 0
sent_packets = 0
last_sent_decision = None
first_send_done = False

def send_vision_decision(decision, confidence=100):
    """decision: 1=GO, 0=STOP. Rate-limited to SEND_INTERVAL."""
    global arduino_seq, last_send_time, sent_packets, last_sent_decision, first_send_done
    now = time.time()
    if now - last_send_time < SEND_INTERVAL:
        return
    last_send_time = now
    arduino_seq = (arduino_seq + 1) & 0xFF
    pkt = bytes([VISION_MAGIC, int(decision) & 0xFF,
                 int(confidence) & 0xFF, arduino_seq])
    try:
        arduino_sock.sendto(pkt, (ARDUINO_IP, ARDUINO_PORT))
        sent_packets += 1
        if not first_send_done:
            first_send_done = True
            print(f"[TX] First vision packet sent → {ARDUINO_IP}:{ARDUINO_PORT}")
            print(f"     (Arduino Serial Monitor should now show [VISION] lines)")
        # Log decision changes only — not every packet
        if decision != last_sent_decision:
            tag = "STOP" if decision == 0 else "GO  "
            print(f"[TX] {tag} → {ARDUINO_IP}:{ARDUINO_PORT} (seq={arduino_seq})")
            last_sent_decision = decision
    except Exception as e:
        print("Arduino send failed:", e)

# =====================================================================
# OPTIONAL YOLO
# =====================================================================
yolo_model = None
if USE_YOLO:
    try:
        from ultralytics import YOLO
        yolo_model = YOLO("yolov8n.pt")
        print("[YOLO] model loaded")
    except Exception as e:
        print("[YOLO] failed to load, falling back to color-only:", e)
        yolo_model = None

# =====================================================================
# VISION
# =====================================================================
KERNEL = np.ones((5, 5), np.uint8)

def analyze(bgr):
    H, W = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
    obstacle = cv2.bitwise_not(yellow)

    y0 = int(H * ROI_TOP_FRAC)
    x0 = int(W * ROI_X1_FRAC)
    x1 = int(W * ROI_X2_FRAC)
    roi_mask = np.zeros_like(obstacle)
    roi_mask[y0:H, x0:x1] = 255
    obstacle = cv2.bitwise_and(obstacle, roi_mask)

    obstacle = cv2.morphologyEx(obstacle, cv2.MORPH_OPEN,  KERNEL)
    obstacle = cv2.morphologyEx(obstacle, cv2.MORPH_CLOSE, KERNEL)

    contours, _ = cv2.findContours(obstacle, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    roi_area = (H - y0) * (x1 - x0)
    min_area = MIN_OBSTACLE_AREA_FRAC * roi_area

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
    decision = 0 if ratio >= STOP_AREA_FRAC else 1
    reason = f"obstacle {ratio*100:.1f}% of corridor" if decision == 0 else "clear"

    if USE_YOLO and YOLO_PERSON_GATE and yolo_model is not None:
        try:
            r = yolo_model.predict(bgr, imgsz=320, conf=0.5,
                                   verbose=False, classes=[0])[0]
            if r.boxes is not None and len(r.boxes) > 0:
                decision = 0
                reason = "person in frame (YOLO)"
        except Exception:
            pass

    return decision, reason, boxes, obstacle, roi_area

# =====================================================================
# STATE
# =====================================================================
frames = {}
last_displayed_fid = -1
frames_shown = 0
last_decision = 1
last_reason = "warming up"
show_mask = False
t0 = time.time()

def purge_stale(cur_fid):
    for k in [k for k in frames if k < cur_fid - 1]:
        del frames[k]

print()
print("Controls:")
print("  [L] toggle ESP32 flash LED")
print("  [1] LED on   [0] LED off")
print("  [M] toggle obstacle mask view")
print("  [Q]/[ESC] quit")
print()

# =====================================================================
# MAIN LOOP
# =====================================================================
while True:
    try:
        data, _ = sock.recvfrom(MAX_PAYLOAD + HDR_SIZE + 64)
    except socket.timeout:
        pass
    else:
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
                    H, W = img.shape[:2]

                    decision, reason, boxes, mask, roi_area = analyze(img)
                    last_decision, last_reason = decision, reason

                    # === Send decision to Arduino ===
                    send_vision_decision(decision, confidence=100)

                    # ---- Overlay ----
                    y0 = int(H * ROI_TOP_FRAC)
                    x0 = int(W * ROI_X1_FRAC)
                    x1 = int(W * ROI_X2_FRAC)
                    cv2.rectangle(img, (x0, y0), (x1, H), (200, 200, 200), 1)

                    for (bx0, by0, bx1, by1, area) in boxes:
                        cv2.rectangle(img, (bx0, by0), (bx1, by1), (0, 0, 255), 2)
                        cv2.putText(img, f"{area/roi_area*100:.1f}%",
                                    (bx0, max(14, by0 - 4)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (0, 0, 255), 1, cv2.LINE_AA)

                    col = (0, 180, 0) if decision == 1 else (0, 0, 220)
                    txt = f"GO   | {reason}" if decision == 1 else f"STOP | {reason}"
                    cv2.rectangle(img, (0, 0), (W, 24), col, -1)
                    cv2.putText(img, txt, (6, 17),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 255, 255), 1, cv2.LINE_AA)

                    # Small footer showing tx status
                    tx_txt = f"TX -> {ARDUINO_IP}:{ARDUINO_PORT}  pkts={sent_packets}"
                    cv2.putText(img, tx_txt, (6, H - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                (255, 255, 255), 1, cv2.LINE_AA)

                    cv2.imshow("ESP32-CAM Obstacle Detection", img)
                    if show_mask:
                        cv2.imshow("Obstacle Mask", mask)

                    last_displayed_fid = fid
                    frames_shown += 1

            del frames[fid]

    # ---- keyboard ----
    key = cv2.waitKey(1) & 0xFF
    if key in (27, ord('q')):
        break
    elif key == ord('l'): send_led_cmd("LED_TOGGLE")
    elif key == ord('1'): send_led_cmd("LED_ON")
    elif key == ord('0'): send_led_cmd("LED_OFF")
    elif key == ord('m'):
        show_mask = not show_mask
        if not show_mask:
            try: cv2.destroyWindow("Obstacle Mask")
            except: pass

    # ---- stats ----
    now = time.time()
    if now - t0 >= STATS_EVERY:
        fps = frames_shown / (now - t0)
        print(f"FPS {fps:5.1f} | decision={last_decision} ({last_reason}) | "
              f"tx_pkts={sent_packets}")
        frames_shown = 0
        t0 = now

cv2.destroyAllWindows()
