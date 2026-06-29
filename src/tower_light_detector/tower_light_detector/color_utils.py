import cv2
import numpy as np
from typing import Tuple, Optional

# Calibrated at exposure_time_absolute=200:
#   붉은색 (red):    H=8,  S=170, V=255
#   주황색 (orange): H=30, S=102, V=197
#
# Detection: ring sample around the brightest point.
# Requires camera exposure preset: v4l2-ctl --set-ctrl=exposure_time_absolute=200

RING_INNER_R = 15
RING_OUTER_R = 45
MIN_V        = 100  # ring pixels dimmer than this are ignored
MIN_PX       = 5


def detect_active_color(frame: np.ndarray,
                        roi: Optional[Tuple[int, int, int, int]] = None
                        ) -> Tuple[str, dict]:
    if roi is not None:
        x0, y0, w, h = roi
        region = frame[y0:y0+h, x0:x0+w]
        offset = (x0, y0)
    else:
        region = frame
        offset = (0, 0)

    hsv  = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)

    _, _, _, max_loc = cv2.minMaxLoc(gray)
    cx, cy = max_loc

    mask = np.zeros(gray.shape, dtype=np.uint8)
    cv2.circle(mask, (cx, cy), RING_OUTER_R, 255, -1)
    cv2.circle(mask, (cx, cy), RING_INNER_R, 0,   -1)

    ring_px = hsv[mask > 0]
    ring_px = ring_px[ring_px[:, 2] > MIN_V]

    bright_loc = (cx + offset[0], cy + offset[1])
    debug = {"bright_loc": bright_loc, "median_hsv": None, "n_pixels": len(ring_px)}

    if len(ring_px) < MIN_PX:
        return "none", debug

    med_h = int(np.median(ring_px[:, 0]))
    med_s = int(np.median(ring_px[:, 1]))
    med_v = int(np.median(ring_px[:, 2]))
    debug["median_hsv"] = (med_h, med_s, med_v)

    # 붉은색: H 낮고(0-15) S 높음(>120)
    if med_h <= 15 and med_s > 120:
        return "red", debug

    # 주황색: H 중간(20-45) S 중간(70-150)
    if 20 <= med_h <= 45 and med_s > 70:
        return "orange", debug

    # 초록색: H 높음(40-90) S > 50
    if 40 <= med_h <= 90 and med_s > 50:
        return "green", debug

    return "none", debug


def draw_debug(frame: np.ndarray, detected: str, debug: dict,
               roi: Optional[Tuple[int, int, int, int]] = None) -> np.ndarray:
    vis = frame.copy()

    COLOR_BGR = {
        "red":    (0, 0, 255),
        "orange": (0, 140, 255),
        "green":  (0, 200, 0),
        "none":   (128, 128, 128),
    }
    c = COLOR_BGR.get(detected, (200, 200, 200))

    bx, by = debug.get("bright_loc", (0, 0))
    cv2.circle(vis, (bx, by), RING_OUTER_R, c, 2)
    cv2.circle(vis, (bx, by), RING_INNER_R, c, 1)

    if roi is not None:
        x0, y0, w, h = roi
        cv2.rectangle(vis, (x0, y0), (x0+w, y0+h), (200, 200, 200), 2)

    label = {"red": "붉은색 (RED)", "orange": "주황색 (ORANGE)",
             "green": "초록색 (GREEN)", "none": "---"}.get(detected, detected)
    cv2.putText(vis, label, (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, c, 3)

    hsv_str = str(debug.get("median_hsv", "-"))
    cv2.putText(vis, f"HSV={hsv_str}", (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 2)

    return vis
