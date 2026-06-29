"""
ROS2 node: tower_light_detector

Reads webcam frames, detects the active color of a tower light (stack light),
and publishes the result so the robot can decide its mission action.

Published topics:
  /tower_light/color   (std_msgs/String)  — "red" | "yellow" | "green" | "none"
  /tower_light/level   (std_msgs/Int8)    —  2=red, 1=yellow, 0=green, -1=none

Parameters (set via YAML or CLI):
  camera_index  (int, default 0)    — /dev/video<N>
  fps           (int, default 10)   — detection rate
  show_debug    (bool, default True) — display OpenCV window
  roi_x, roi_y, roi_w, roi_h        — region of interest in pixels (0 = full frame)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8

import cv2
from .color_utils import detect_active_color, draw_debug


LEVEL_MAP = {"red": 2, "yellow": 1, "green": 0, "none": -1}


class TowerLightDetector(Node):
    def __init__(self):
        super().__init__("tower_light_detector")

        # Parameters
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("fps", 10)
        self.declare_parameter("show_debug", True)
        self.declare_parameter("roi_x", 0)
        self.declare_parameter("roi_y", 0)
        self.declare_parameter("roi_w", 0)
        self.declare_parameter("roi_h", 0)

        cam_idx   = self.get_parameter("camera_index").value
        fps       = self.get_parameter("fps").value
        self.show = self.get_parameter("show_debug").value

        roi_x = self.get_parameter("roi_x").value
        roi_y = self.get_parameter("roi_y").value
        roi_w = self.get_parameter("roi_w").value
        roi_h = self.get_parameter("roi_h").value
        self.roi = (roi_x, roi_y, roi_w, roi_h) if roi_w > 0 and roi_h > 0 else None

        # Publishers
        self.pub_color = self.create_publisher(String, "/tower_light/color", 10)
        self.pub_level = self.create_publisher(Int8,   "/tower_light/level", 10)

        # Camera
        self.cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera index {cam_idx}")
            raise RuntimeError("Camera open failed")
        # Fix exposure to prevent overexposure of tower light
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # manual mode
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 200)
        self.get_logger().info(f"Camera {cam_idx} opened — detecting at {fps} Hz")

        self._prev_color = ""
        self.create_timer(1.0 / fps, self._timer_cb)

    def _timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Frame capture failed")
            return

        detected, dbg = detect_active_color(frame, self.roi)

        # Publish only when color changes (reduces unnecessary traffic)
        if detected != self._prev_color:
            self.get_logger().info(f"Tower light → {detected.upper()}  (scores: {dbg['scores']})")
            self._prev_color = detected

        color_msg = String()
        color_msg.data = detected
        self.pub_color.publish(color_msg)

        level_msg = Int8()
        level_msg.data = LEVEL_MAP[detected]
        self.pub_level.publish(level_msg)

        if self.show:
            vis = draw_debug(frame, detected, dbg["scores"], self.roi)
            cv2.imshow("Tower Light Detector", vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TowerLightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
