"""
탑라이트(산업용 3색 신호등) 색상 감지 노드
- /camera/image_raw 구독 → HSV 마스킹으로 RED/YELLOW/GREEN 판별
- /toplight/color (String) 퍼블리시
- /toplight/debug_image (Image) 퍼블리시 (마스킹 결과 시각화)
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


# ── HSV 범위 (OpenCV: H=0~179, S=0~255, V=0~255) ──────────────────────
# 현장 조명에 따라 파라미터로 조정 가능
HSV_RANGES = {
    'RED': [
        (np.array([0, 120, 80]),   np.array([10, 255, 255])),   # 빨강 낮은 H
        (np.array([165, 120, 80]), np.array([179, 255, 255])),  # 빨강 높은 H
    ],
    'YELLOW': [
        (np.array([15, 100, 80]),  np.array([35, 255, 255])),   # 주황/노랑
    ],
    'GREEN': [
        (np.array([40, 80, 80]),   np.array([85, 255, 255])),   # 초록
    ],
}

COLOR_BGR = {
    'RED':    (0, 0, 255),
    'YELLOW': (0, 200, 255),
    'GREEN':  (0, 255, 0),
    'NONE':   (128, 128, 128),
}


class ToplightDetector(Node):
    def __init__(self):
        super().__init__('toplight_detector')

        # ── 파라미터 ──
        self.image_topic   = self.declare_parameter('image_topic', '/camera/image_raw').value
        self.min_area      = self.declare_parameter('min_area', 300).value       # 최소 감지 픽셀 면적
        self.roi_top_frac  = self.declare_parameter('roi_top_frac', 0.0).value   # ROI 상단 비율
        self.roi_bot_frac  = self.declare_parameter('roi_bot_frac', 0.5).value   # ROI 하단 비율 (화면 위쪽 절반만)

        self.bridge = CvBridge()
        self.last_color = 'NONE'

        # ── 구독 ──
        self.create_subscription(Image, self.image_topic, self._image_callback, 10)

        # ── 발행 ──
        self.color_pub = self.create_publisher(String, '/toplight/color', 10)
        self.debug_pub = self.create_publisher(Image, '/toplight/debug_image', 10)

        self.get_logger().info(
            f'ToplightDetector 시작. 구독: {self.image_topic} | '
            f'ROI: 상단 {self.roi_top_frac:.0%} ~ 하단 {self.roi_bot_frac:.0%}'
        )

    def _image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        # ── ROI 적용 (탑라이트는 보통 화면 위쪽에 있음) ──
        y0 = int(h * self.roi_top_frac)
        y1 = int(h * self.roi_bot_frac)
        roi = frame[y0:y1, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        detected_color, areas = self._detect_color(hsv)

        # 상태 변화 시만 로그
        if detected_color != self.last_color:
            self.get_logger().info(
                f'탑라이트: {self.last_color} → {detected_color} '
                f'(areas: {areas})'
            )
            self.last_color = detected_color

        # 퍼블리시
        msg_out = String()
        msg_out.data = detected_color
        self.color_pub.publish(msg_out)

        # 디버그 이미지
        debug = self._draw_debug(frame, roi, y0, detected_color, areas)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

    def _detect_color(self, hsv: np.ndarray) -> tuple[str, dict]:
        """각 색상 마스크 픽셀 면적 계산 → 가장 큰 색 반환"""
        areas: dict[str, int] = {}

        for color, ranges in HSV_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, lo, hi)

            # 노이즈 제거
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

            area = int(np.sum(mask > 0))
            areas[color] = area

        best_color = max(areas, key=lambda c: areas[c])
        return (best_color if areas[best_color] >= self.min_area else 'NONE'), areas

    def _draw_debug(
        self,
        frame: np.ndarray,
        roi: np.ndarray,
        y0: int,
        detected: str,
        areas: dict,
    ) -> np.ndarray:
        debug = frame.copy()

        # ROI 경계선
        cv2.rectangle(debug, (0, y0), (frame.shape[1], y0 + roi.shape[0]),
                      (255, 255, 0), 2)

        # 감지된 색상 텍스트
        bgr = COLOR_BGR.get(detected, (128, 128, 128))
        cv2.putText(debug, f'TOPLIGHT: {detected}', (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, bgr, 3)

        # 각 색 면적 표시
        for i, (color, area) in enumerate(areas.items()):
            cv2.putText(debug, f'{color}: {area}px', (10, 80 + i * 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_BGR[color], 2)

        return debug


def main(args=None):
    rclpy.init(args=args)
    node = ToplightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
