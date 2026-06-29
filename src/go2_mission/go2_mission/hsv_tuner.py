"""
HSV 실시간 튜너
- ROS2 카메라 토픽 구독 → OpenCV 창에 슬라이더
- 마스크 결과 실시간 확인 → 's' 누르면 현재 값 출력
- 사용: ros2 run go2_mission hsv_tuner --ros-args -p image_topic:=/camera/image_raw
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

WINDOW = 'HSV Tuner'


class HsvTuner(Node):
    def __init__(self):
        super().__init__('hsv_tuner')
        self.image_topic = self.declare_parameter(
            'image_topic', '/camera/image_raw'
        ).value

        self.bridge = CvBridge()
        self.frame: np.ndarray | None = None

        self.create_subscription(Image, self.image_topic, self._cb, 10)

        # OpenCV 창 + 슬라이더
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 1280, 600)

        def _tb(name, val, max_val=255):
            cv2.createTrackbar(name, WINDOW, val, max_val, lambda _: None)

        _tb('H min',  0,   179)
        _tb('H max', 30,   179)
        _tb('S min', 100, 255)
        _tb('S max', 255, 255)
        _tb('V min', 80,  255)
        _tb('V max', 255, 255)
        # 빨강은 H가 0~10 AND 165~179 두 범위라 두 번째 범위 슬라이더 추가
        _tb('H2 min', 165, 179)
        _tb('H2 max', 179, 179)
        _tb('Use H2 (red)', 0, 1)   # 0=단일범위, 1=빨강 두 범위 합산

        self.create_timer(0.033, self._render)  # ~30Hz
        self.get_logger().info(
            f'HSV Tuner 시작. 구독: {self.image_topic}\n'
            '  s = 현재 HSV 값 터미널 출력\n'
            '  q = 종료'
        )

    def _cb(self, msg: Image):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _render(self):
        if self.frame is None:
            return

        frame = self.frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        def tb(name):
            return cv2.getTrackbarPos(name, WINDOW)

        lo  = np.array([tb('H min'), tb('S min'), tb('V min')])
        hi  = np.array([tb('H max'), tb('S max'), tb('V max')])
        mask = cv2.inRange(hsv, lo, hi)

        if tb('Use H2 (red)') == 1:
            lo2  = np.array([tb('H2 min'), tb('S min'), tb('V min')])
            hi2  = np.array([tb('H2 max'), tb('S max'), tb('V max')])
            mask |= cv2.inRange(hsv, lo2, hi2)

        # 노이즈 제거
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)

        # 윤곽선 + 면적 표시
        vis = frame.copy()
        contours, _ = cv2.findContours(
            mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        total_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 50:
                continue
            total_area += area
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(vis, f'{int(area)}px', (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.putText(vis, f'total={total_area}px  contours={len(contours)}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # 마스크를 3채널로 변환해서 원본과 나란히
        mask_bgr = cv2.cvtColor(mask_clean, cv2.COLOR_GRAY2BGR)
        combined = np.hstack([vis, mask_bgr])
        cv2.imshow(WINDOW, combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            self._print_values(tb)
        elif key == ord('q'):
            raise KeyboardInterrupt

    def _print_values(self, tb):
        use_h2 = tb('Use H2 (red)') == 1
        print('\n===== HSV 튜닝 결과 (toplight_detector.py 에 복사) =====')
        print(f"HSV_RANGES = {{")
        if use_h2:
            print(f"    'RED': [")
            print(f"        (np.array([{tb('H min')}, {tb('S min')}, {tb('V min')}]), "
                  f"np.array([{tb('H max')}, {tb('S max')}, {tb('V max')}])),")
            print(f"        (np.array([{tb('H2 min')}, {tb('S min')}, {tb('V min')}]), "
                  f"np.array([{tb('H2 max')}, {tb('S max')}, {tb('V max')}])),")
            print(f"    ],")
        else:
            print(f"    'COLOR': [")
            print(f"        (np.array([{tb('H min')}, {tb('S min')}, {tb('V min')}]), "
                  f"np.array([{tb('H max')}, {tb('S max')}, {tb('V max')}])),")
            print(f"    ],")
        print("}")
        print('=========================================================\n')


def main(args=None):
    rclpy.init(args=args)
    node = HsvTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
