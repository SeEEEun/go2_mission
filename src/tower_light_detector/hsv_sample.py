import cv2
import numpy as np

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
print("탑 라이트를 카메라에 비추세요. 제일 밝은 픽셀의 HSV를 출력합니다.")
print("q 누르면 종료\n")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 가장 밝은 픽셀 위치
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, _, _, max_loc = cv2.minMaxLoc(gray)
    x, y = max_loc

    h, s, v = hsv[y, x]
    b, g, r = frame[y, x]

    print(f"\r밝은점 ({x:4d},{y:4d}) | HSV=({h:3d},{s:3d},{v:3d}) | BGR=({b:3d},{g:3d},{r:3d})", end="", flush=True)

    # 밝은 점 표시
    cv2.circle(frame, (x, y), 10, (0, 255, 0), 2)
    cv2.putText(frame, f"H={h} S={s} V={v}", (x+15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
    cv2.imshow("HSV Sampler", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
