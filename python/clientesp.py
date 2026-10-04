import socket, time, sys
import numpy as np
import cv2

LISTEN_PORT = 4210
MAX_PAYLOAD = 1400
HDR_SIZE    = 4
STATS_EVERY = 5.0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)   # big kernel buffer
sock.bind(("0.0.0.0", LISTEN_PORT))
sock.settimeout(0.5)
print(f"Listening on UDP :{LISTEN_PORT}")

# frames[fid] = {pkt_idx: bytes}
frames = {}
last_displayed_fid = -1

frames_shown = 0
frames_dropped = 0
t0 = time.time()

def purge_stale(current_fid):
    """Drop any frame older than (current_fid - 1) — never wait for stale data."""
    for k in [k for k in frames if k < current_fid - 1]:
        del frames[k]

while True:
    try:
        data, _ = sock.recvfrom(MAX_PAYLOAD + HDR_SIZE + 64)
    except socket.timeout:
        continue

    if len(data) < HDR_SIZE + 1:
        continue

    fid   = (data[0] << 8) | data[1]
    pidx  =  data[2]
    total =  data[3]
    payload = data[HDR_SIZE:]

    # New frame arrived — opportunistically drop anything much older
    purge_stale(fid)

    bucket = frames.setdefault(fid, {})
    bucket[pidx] = payload

    if len(bucket) == total:
        # Only bother decoding if we haven't already shown a newer frame
        if fid > last_displayed_fid:
            jpg = b"".join(bucket[i] for i in range(total))
            arr = np.frombuffer(jpg, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is not None:
                cv2.imshow("ESP32-CAM Realtime", img)
                cv2.waitKey(1)
                last_displayed_fid = fid
                frames_shown += 1
        del frames[fid]

    # --------- stats ---------
    now = time.time()
    if now - t0 >= STATS_EVERY:
        fps = frames_shown / (now - t0)
        print(f"FPS: {fps:5.1f}   pending frames: {len(frames)}   "
              f"last fid: {last_displayed_fid}")
        frames_shown = 0
        t0 = now
