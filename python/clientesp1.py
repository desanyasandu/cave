import socket, time
import numpy as np
import cv2

# ---------------- CONFIG ----------------
ESP32_IP    = "10.160.191.130"   # <-- ESP32's IP from Serial Monitor
LISTEN_PORT = 4210
CTRL_PORT   = 4211
MAX_PAYLOAD = 1400
HDR_SIZE    = 4
STATS_EVERY = 2.0

# ---- Yellow surface HSV range (tune these from calibrate.py) ----
LOWER_YELLOW = np.array([18, 60, 60])
UPPER_YELLOW = np.array([45, 255, 255])

# ---- Region of interest: forward corridor ----
# Ignore the top of the frame (walls, background, chair)
ROI_TOP_FRAC = 0.40      # 0 = whole frame, 0.40 = bottom 60%
ROI_X1_FRAC  = 0.15
ROI_X2_FRAC  = 0.85

# ---- Obstacle size filter ----
# An obstacle must occupy at least this fraction of the ROI area to count
MIN_OBSTACLE_AREA_FRAC = 0.008    # ~0.8% of ROI area

# ---- Decision logic ----
# If total obstacle area in the forward corridor exceeds this,
# declare STOP
STOP_AREA_FRAC = 0.030            # ~3% of ROI area

# ---------------- SETUP ----------------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
sock.bind(("0.0.0.0", LISTEN_PORT))
sock.settimeout(0.3)

ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def send_cmd(cmd: str):
    try: ctrl_sock.sendto(cmd.encode(), (ESP32_IP, CTRL_PORT))
    except Exception as e: print("ctrl err:", e)

# ---------------- VISION ----------------
KERNEL = np.ones((5, 5), np.uint8)

def analyze(bgr):
    H, W = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
    obstacle = cv2.bitwise_not(yellow)

    # Restrict to forward ROI
    y0 = int(H * ROI_TOP_FRAC)
    x0 = int(W * ROI_X1_FRAC)
    x1 = int(W * ROI_X2_FRAC)
    roi_mask = np.zeros_like(obstacle)
    roi_mask[y0:H, x0:x1] = 255
    obstacle = cv2.bitwise_and(obstacle, roi_mask)

    # Morphological cleanup — removes speckle noise, fills holes
    obstacle = cv2.morphologyEx(obstacle, cv2.MORPH_OPEN,  KERNEL)
    obstacle = cv2.morphologyEx(obstacle, cv2.MORPH_CLOSE, KERNEL)

    # Find contours and filter by area
    contours, _ = cv2.findContours(obstacle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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

    ratio = total_obs_area / roi_area
    decision = 0 if ratio >= STOP_AREA_FRAC else 1
    reason   = f"obstacle {ratio*100:.1f}% of corridor" if decision == 0 else "clear"
    return decision, reason, boxes, obstacle, roi_area

# ---------------- STATE ----------------
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

print("\nControls:  [L] flash LED   [1] on   [0] off   "
      "[M] toggle mask view   [Q]/[ESC] quit\n")

# ---------------- MAIN LOOP ----------------
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

                    # ---- Overlay ----
                    # ROI box
                    y0 = int(H * ROI_TOP_FRAC)
                    x0 = int(W * ROI_X1_FRAC)
                    x1 = int(W * ROI_X2_FRAC)
                    cv2.rectangle(img, (x0, y0), (x1, H), (200, 200, 200), 1)

                    # Obstacle boxes
                    for (bx0, by0, bx1, by1, area) in boxes:
                        cv2.rectangle(img, (bx0, by0), (bx1, by1), (0, 0, 255), 2)
                        cv2.putText(img, f"{area/roi_area*100:.1f}%",
                                    (bx0, max(14, by0 - 4)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

                    # Banner
                    col = (0, 180, 0) if decision == 1 else (0, 0, 220)
                    txt = f"GO   | {reason}" if decision == 1 else f"STOP | {reason}"
                    cv2.rectangle(img, (0, 0), (W, 24), col, -1)
                    cv2.putText(img, txt, (6, 17), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (255, 255, 255), 1, cv2.LINE_AA)

                    cv2.imshow("ESP32-CAM Obstacle Detection", img)
                    if show_mask:
                        cv2.imshow("Obstacle Mask", mask)

                    last_displayed_fid = fid
                    frames_shown += 1

                    # Optional: LED as stop indicator
                    # send_cmd("LED_ON" if decision == 0 else "LED_OFF")

            del frames[fid]

    # ---- keyboard ----
    key = cv2.waitKey(1) & 0xFF
    if key in (27, ord('q')):
        break
    elif key == ord('l'): send_cmd("LED_TOGGLE")
    elif key == ord('1'): send_cmd("LED_ON")
    elif key == ord('0'): send_cmd("LED_OFF")
    elif key == ord('m'):
        show_mask = not show_mask
        if not show_mask:
            cv2.destroyWindow("Obstacle Mask")

    # ---- stats ----
    now = time.time()
    if now - t0 >= STATS_EVERY:
        fps = frames_shown / (now - t0)
        print(f"FPS {fps:5.1f} | decision={last_decision} | {last_reason}")
        frames_shown = 0
        t0 = now

cv2.destroyAllWindows()
