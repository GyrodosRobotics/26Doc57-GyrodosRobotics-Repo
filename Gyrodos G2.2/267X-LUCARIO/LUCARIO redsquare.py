import cv2
import numpy as np

mtx = np.array([[5.74798166e+02, 0.00000000e+00, 1.01247178e+03],
                [0.00000000e+00, 5.83712017e+02, 5.76989155e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

dist = np.array([[-0.19427912, 0.03186565, 0.00039767, 0.00129534, -0.0020267]])

SMOOTH = 0.1

# ── Fixed output window size (4:3) ────────────────────────────────────────────
OUTPUT_W = 1024
OUTPUT_H = 768

def undistort_frame(frame, mtx, dist):
    h, w = frame.shape[:2]
    new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    undistorted = cv2.undistort(frame, mtx, dist, None, new_camera_mtx)
    x, y, w, h = roi
    undistorted = undistorted[y:y+h, x:x+w]
    return undistorted


def find_red_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0,   40,  40])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160,  40,  40])
    upper_red2 = np.array([180, 255, 255])

    mask = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red1, upper_red1),
        cv2.inRange(hsv, lower_red2, upper_red2)
    )

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def find_square_edges(frame, mask, debug):
    fh, fw = frame.shape[:2]

    diag = np.sqrt(fw**2 + fh**2)
    min_line_len = int(diag * 0.05)
    max_line_gap = int(diag * 0.02)

    edges = cv2.Canny(mask, 50, 150)
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=30,
        minLineLength=min_line_len,
        maxLineGap=max_line_gap
    )

    if lines is None:
        cv2.putText(debug, "No lines found", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return None

    horizontals = []
    verticals   = []

    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = np.degrees(np.arctan2(abs(y2 - y1), abs(x2 - x1)))

        cv2.line(debug, (x1, y1), (x2, y2), (180, 180, 180), 1, cv2.LINE_AA)

        if angle < 20:
            y_mid = (y1 + y2) // 2
            x_s   = min(x1, x2)
            x_e   = max(x1, x2)
            horizontals.append((y_mid, x_s, x_e))
            cv2.line(debug, (x1, y1), (x2, y2), (0, 255, 255), 2, cv2.LINE_AA)

        elif angle > 70:
            x_mid = (x1 + x2) // 2
            y_s   = min(y1, y2)
            y_e   = max(y1, y2)
            verticals.append((x_mid, y_s, y_e))
            cv2.line(debug, (x1, y1), (x2, y2), (255, 0, 255), 2, cv2.LINE_AA)

    def cluster_lines(lines_1d, idx=0):
        if not lines_1d:
            return []
        lines_1d = sorted(lines_1d, key=lambda l: l[idx])
        clusters = []
        current  = [lines_1d[0]]

        for line in lines_1d[1:]:
            if abs(line[idx] - current[-1][idx]) < int(diag * 0.04):
                current.append(line)
            else:
                clusters.append(current)
                current = [line]
        clusters.append(current)

        merged = []
        for cluster in clusters:
            pos   = int(np.mean([l[idx] for l in cluster]))
            start = int(np.min( [l[1]   for l in cluster]))
            end   = int(np.max( [l[2]   for l in cluster]))
            merged.append((pos, start, end))
        return merged

    h_lines = cluster_lines(horizontals, idx=0)
    v_lines = cluster_lines(verticals,   idx=0)

    for (y, xs, xe) in h_lines:
        cv2.line(debug, (xs, y), (xe, y), (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(debug, f"H y={y}", (xs, y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    for (x, ys, ye) in v_lines:
        cv2.line(debug, (x, ys), (x, ye), (255, 128, 0), 2, cv2.LINE_AA)
        cv2.putText(debug, f"V x={x}", (x + 4, ys + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 128, 0), 1)

    if not h_lines or not v_lines:
        cv2.putText(debug, f"Not enough lines  H={len(h_lines)} V={len(v_lines)}",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return None

    y_positions = sorted([l[0] for l in h_lines])
    x_positions = sorted([l[0] for l in v_lines])

    if len(y_positions) >= 2:
        y_top    = y_positions[0]
        y_bottom = y_positions[-1]
    else:
        y_top    = max(0,  y_positions[0] - fw // 4)
        y_bottom = min(fh, y_positions[0] + fw // 4)

    if len(x_positions) >= 2:
        x_left  = x_positions[0]
        x_right = x_positions[-1]
    else:
        x_left  = max(0,  x_positions[0] - fh // 4)
        x_right = min(fw, x_positions[0] + fh // 4)

    cx = (x_left + x_right) // 2
    cy = (y_top  + y_bottom) // 2
    w  = x_right - x_left
    h  = y_bottom - y_top

    # ── Orange bounding box ───────────────────────────────────────────────────
    cv2.rectangle(debug, (x_left, y_top), (x_right, y_bottom), (0, 128, 255), 2)
    cv2.putText(debug, f"cx={cx} cy={cy} w={w} h={h}",
                (x_left, y_top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 255), 2)

    # ── Shrinking debug boxes ─────────────────────────────────────────────────
    scales = [1.0,          0.90,           0.80,          0.70,          0.60]
    colors = [(200,200,200),(180, 180, 0), (0, 200, 200), (0, 180, 0),  (0, 255, 255)]

    for scale, color in zip(scales, colors):
        bw  = int(w * scale)
        bh  = int(h * scale)
        bx1 = cx - bw // 2
        by1 = cy - bh // 2
        cv2.rectangle(debug, (bx1, by1), (bx1 + bw, by1 + bh), color, 1)

    # ── Actual 4:3 crop box on debug ─────────────────────────────────────────
    crop_w = int(w * 0.80)
    crop_h = int(h * 0.60)
    cx1 = cx - crop_w // 2
    cy1 = cy - crop_h // 2
    cv2.rectangle(debug, (cx1, cy1), (cx1 + crop_w, cy1 + crop_h), (0, 255, 255), 2)
    cv2.putText(debug, "4:3", (cx1 + 4, cy1 + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    return cx, cy, w, h


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # ── Create windows once at fixed sizes ───────────────────────────────────
    cv2.namedWindow("Camera Output (4:3)", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera Output (4:3)", OUTPUT_W, OUTPUT_H)

    cv2.namedWindow("Debug View", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Debug View", 800, 600)

    cv2.namedWindow("Red Mask", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Red Mask", 400, 300)

    smooth_cx = None
    smooth_cy = None
    smooth_w  = None
    smooth_h  = None

    # ── Searching placeholder — shown in the output window until detected ─────
    placeholder = np.zeros((OUTPUT_H, OUTPUT_W, 3), dtype=np.uint8)
    cv2.putText(placeholder, "Searching for square...",
                (OUTPUT_W // 2 - 200, OUTPUT_H // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    cv2.imshow("Camera Output (4:3)", placeholder)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        # ── 1. Undistort ──────────────────────────────────────────────────────
        frame = undistort_frame(frame, mtx, dist)
        fh, fw = frame.shape[:2]
        debug  = frame.copy()

        # ── 2. Red mask ───────────────────────────────────────────────────────
        mask = find_red_mask(frame)

        # ── 3. Find square edges ──────────────────────────────────────────────
        result = find_square_edges(frame, mask, debug)

        if result is not None:
            target_cx, target_cy, target_w, target_h = result

            if smooth_cx is None:
                smooth_cx = float(target_cx)
                smooth_cy = float(target_cy)
                smooth_w  = float(target_w)
                smooth_h  = float(target_h)
            else:
                smooth_cx += (target_cx - smooth_cx) * SMOOTH
                smooth_cy += (target_cy - smooth_cy) * SMOOTH
                smooth_w  += (target_w  - smooth_w)  * SMOOTH
                smooth_h  += (target_h  - smooth_h)  * SMOOTH

        # ── 4. Crop, upscale to fixed output size ─────────────────────────────
        if smooth_cx is not None:
            cx = int(smooth_cx)
            cy = int(smooth_cy)

            crop_w = int(smooth_w * 0.80)
            crop_h = int(smooth_h * 0.60)

            x1 = max(0, min(cx - crop_w // 2, fw - crop_w))
            y1 = max(0, min(cy - crop_h // 2, fh - crop_h))
            x2 = x1 + crop_w
            y2 = y1 + crop_h

            cropped = frame[y1:y2, x1:x2]

            # ── Upscale to fixed OUTPUT_W x OUTPUT_H ─────────────────────────
            output = cv2.resize(cropped, (OUTPUT_W, OUTPUT_H),
                                interpolation=cv2.INTER_LINEAR)

            cv2.imshow("Camera Output (4:3)", output)

        cv2.imshow("Debug View", debug)
        cv2.imshow("Red Mask",   mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
