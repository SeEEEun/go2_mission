"""
ROS2 node: mission_dispatcher

Subscribes to:
  /tower_light/color  (std_msgs/String)  — from detector_node
  /competition/section (std_msgs/Int8)   — current section number (from ops team or manual)

Publishes:
  /mission/action  (std_msgs/String)  — action string for the robot controller

Mission rules (from competition spec):
  Section 2:
    yellow → "TOUCH_LEFT_WALL"
    red    → "TOUCH_RIGHT_WALL"
  Section 3:
    yellow → "QR_RIGHT_WALL"
    red    → "QR_LEFT_WALL"
  Other sections: extend MISSION_TABLE as needed.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8


MISSION_TABLE = {
    2: {"orange": "TOUCH_LEFT_WALL",  "red": "TOUCH_RIGHT_WALL"},
    3: {"orange": "QR_RIGHT_WALL",    "red": "QR_LEFT_WALL"},
    # add more sections here
}


class MissionDispatcher(Node):
    def __init__(self):
        super().__init__("mission_dispatcher")

        self.current_section = -1
        self.current_color   = "none"

        self.sub_color   = self.create_subscription(String, "/tower_light/color",    self._color_cb,   10)
        self.sub_section = self.create_subscription(Int8,   "/competition/section",  self._section_cb, 10)
        self.pub_action  = self.create_publisher(String, "/mission/action", 10)

        self.get_logger().info("Mission dispatcher ready. Waiting for light color and section info.")

    def _section_cb(self, msg: Int8):
        self.current_section = msg.data
        self.get_logger().info(f"Section updated → {self.current_section}")
        self._dispatch()

    def _color_cb(self, msg: String):
        self.current_color = msg.data
        self._dispatch()

    def _dispatch(self):
        section = self.current_section
        color   = self.current_color

        if section < 0 or color in ("none", "green"):
            return

        section_missions = MISSION_TABLE.get(section)
        if section_missions is None:
            self.get_logger().warn(f"No mission table entry for section {section}")
            return

        action = section_missions.get(color)
        if action is None:
            self.get_logger().warn(f"No action defined for section={section}, color={color}")
            return

        out = String()
        out.data = action
        self.pub_action.publish(out)
        self.get_logger().info(f"MISSION DISPATCH → section={section}, color={color} → {action}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionDispatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
