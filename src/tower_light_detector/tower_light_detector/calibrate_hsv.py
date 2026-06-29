"""
Standalone HSV calibration tool — run BEFORE the competition to tune thresholds.

Usage:
  python3 calibrate_hsv.py [camera_index]

Controls:
  Click on the lit tower light in the preview window to sample its HSV.
  Press 'q' to quit.
"""

import sys
import cv2
import numpy as np


def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_frame = param["hsv"]
        h, s, v = hsv_frame[y, x]
        print(f"  Clicked ({x},{y}) → HSV=({h}, {s}, {v})")
        print(f"  Suggested range: lower=({max(0,h-10)},{max(0,s-40)},{max(0,v-40)})  "
              f"upper=({min(180,h+10)},255,255)")


def main():
    cam_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"Cannot open camera {cam_idx}")
        return

    print("Click on the lit tower light to sample its HSV value.")
    print("Press 'q' to quit.\n")

    param = {"hsv": None}
    cv2.namedWindow("HSV Calibration")
    cv2.setMouseCallback("HSV Calibration", mouse_callback, param)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        param["hsv"] = hsv

        cv2.putText(frame, "Click on lit light to sample HSV", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("HSV Calibration", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
