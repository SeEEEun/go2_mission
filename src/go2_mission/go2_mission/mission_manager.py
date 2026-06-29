"""
ICROS 2026 미션 매니저
- /factory/diagnostics  → 다음 목표 구역
- /trg/goal             → TRG Planner 경로 생성
- /odom                 → 거리 기반 도착 감지 (1차)
- /toplight/color       → 탑라이트 GREEN 확인으로 도착 이중 확인 (2차)
- /mission/trigger      → 버튼/사진 미션 트리거
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class State(Enum):
    IDLE           = auto()  # 다음 목표 대기
    NAVIGATING     = auto()  # TRG Planner로 이동 중
    NEAR_ZONE      = auto()  # 거리 도달 → 탑라이트 GREEN 대기
    ARRIVED        = auto()  # 탑라이트 확인 완료 → 미션 시작
    MISSION        = auto()  # 미션 수행 중
    MISSION_DONE   = auto()  # 미션 완료 → 다음 목표 대기
    FINISHED       = auto()  # 경기 완료


DEFAULT_ZONE_POSITIONS = {
    1: (0.0,  0.0,  0.0),   # ① 8시 방향 (출발점, 맵 원점)
    2: (-3.0, 2.5,  0.0),   # ② 10시 방향
    3: (3.0,  2.5,  0.0),   # ③ 2시 방향
    4: (3.0,  -2.5, 0.0),   # ④ 4시 방향
}

DEVICE_TO_ZONE = {
    'device1': 1,
    'device2': 2,
    'device3': 3,
    'device4': 4,
}


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # ── 파라미터 ──
        self._declare_zone_params()
        self.arrive_threshold   = self.declare_parameter('arrive_threshold', 0.6).value
        self.mission_timeout    = self.declare_parameter('mission_timeout', 15.0).value
        self.use_toplight       = self.declare_parameter('use_toplight', True).value
        self.toplight_timeout   = self.declare_parameter('toplight_timeout', 10.0).value  # GREEN 안 오면 포기 시간(초)

        self.zone_positions = self._load_zone_params()

        # ── 상태 ──
        self.state          = State.IDLE
        self.current_zone: int | None = None
        self.current_x      = 0.0
        self.current_y      = 0.0
        self.toplight_color = 'NONE'
        self.near_zone_time: float | None = None   # NEAR_ZONE 진입 시각
        self.mission_start_time: float | None = None

        # ── QoS ──
        diag_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── 구독 ──
        self.create_subscription(DiagnosticArray, '/factory/diagnostics',
                                 self._diag_callback, diag_qos)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(String, '/toplight/color',
                                 self._toplight_callback, 10)

        # ── 발행 ──
        self.goal_pub           = self.create_publisher(PoseStamped, '/trg/goal', 10)
        self.status_pub         = self.create_publisher(String, '/mission/status', 10)
        self.mission_trigger_pub = self.create_publisher(Bool, '/mission/trigger', 10)

        # ── 10 Hz 상태 루프 ──
        self.create_timer(0.1, self._state_loop)

        self.get_logger().info(
            f'MissionManager 시작 | use_toplight={self.use_toplight} '
            f'arrive_threshold={self.arrive_threshold}m'
        )

    # ─────────────────────────────────────────────
    # 파라미터
    # ─────────────────────────────────────────────
    def _declare_zone_params(self):
        for zone, (x, y, yaw) in DEFAULT_ZONE_POSITIONS.items():
            self.declare_parameter(f'zone{zone}_x',   x)
            self.declare_parameter(f'zone{zone}_y',   y)
            self.declare_parameter(f'zone{zone}_yaw', yaw)

    def _load_zone_params(self) -> dict[int, tuple]:
        return {
            zone: (
                self.get_parameter(f'zone{zone}_x').value,
                self.get_parameter(f'zone{zone}_y').value,
                self.get_parameter(f'zone{zone}_yaw').value,
            )
            for zone in DEFAULT_ZONE_POSITIONS
        }

    # ─────────────────────────────────────────────
    # 콜백
    # ─────────────────────────────────────────────
    def _diag_callback(self, msg: DiagnosticArray):
        if self.state not in (State.IDLE, State.MISSION_DONE):
            return

        for status in msg.status:
            zone = DEVICE_TO_ZONE.get(status.name)
            if zone is None:
                continue
            if status.level == 0 and zone != self.current_zone:
                self.get_logger().info(
                    f'새 목표: {status.name} → 구역 {zone} ("{status.message}")'
                )
                self._start_navigation(zone)
                break

    def _odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def _toplight_callback(self, msg: String):
        self.toplight_color = msg.data

    # ─────────────────────────────────────────────
    # 상태 머신
    # ─────────────────────────────────────────────
    def _state_loop(self):
        if self.state == State.NAVIGATING:
            self._check_near_zone()

        elif self.state == State.NEAR_ZONE:
            self._check_toplight_confirm()

        elif self.state == State.ARRIVED:
            self._execute_mission()

        elif self.state == State.MISSION:
            self._check_mission_done()

    # ─────────────────────────────────────────────
    # 내비게이션
    # ─────────────────────────────────────────────
    def _start_navigation(self, zone: int):
        self.current_zone = zone
        self.state = State.NAVIGATING
        self._publish_goal(zone)
        self._publish_status(f'NAVIGATING_TO_ZONE_{zone}')
        self.get_logger().info(f'구역 {zone}으로 이동.')

    def _publish_goal(self, zone: int):
        x, y, yaw = self.zone_positions[zone]
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(goal)

    def _check_near_zone(self):
        if self.current_zone is None:
            return
        x, y, _ = self.zone_positions[self.current_zone]
        dist = math.hypot(self.current_x - x, self.current_y - y)
        if dist < self.arrive_threshold:
            self.get_logger().info(
                f'구역 {self.current_zone} 근접 (dist={dist:.3f}m). '
                f'탑라이트 GREEN {"대기" if self.use_toplight else "스킵"}.'
            )
            self.state = State.NEAR_ZONE
            self.near_zone_time = self._now()

    def _check_toplight_confirm(self):
        if not self.use_toplight:
            self._on_arrived()
            return

        if self.toplight_color == 'GREEN':
            self.get_logger().info(f'탑라이트 GREEN 확인 → 구역 {self.current_zone} 도착.')
            self._on_arrived()
            return

        # toplight_timeout 초 내 GREEN 없으면 그냥 진행
        elapsed = self._now() - (self.near_zone_time or self._now())
        if elapsed >= self.toplight_timeout:
            self.get_logger().warn(
                f'탑라이트 GREEN 미확인 ({elapsed:.1f}s 경과). '
                f'현재={self.toplight_color}. 도착으로 간주.'
            )
            self._on_arrived()

    def _on_arrived(self):
        self.state = State.ARRIVED
        self.near_zone_time = None
        self._publish_status(f'ARRIVED_ZONE_{self.current_zone}')

    # ─────────────────────────────────────────────
    # 미션
    # ─────────────────────────────────────────────
    def _execute_mission(self):
        self.get_logger().info(f'구역 {self.current_zone} 미션 시작.')
        self.state = State.MISSION
        self.mission_start_time = self._now()

        trigger = Bool()
        trigger.data = True
        self.mission_trigger_pub.publish(trigger)
        self._publish_status(f'MISSION_ZONE_{self.current_zone}')

    def _check_mission_done(self):
        if self.mission_start_time is None:
            return

        # 탑라이트 RED → 미션 완료 신호로 해석 (선택적 사용)
        if self.use_toplight and self.toplight_color == 'RED':
            self.get_logger().info(
                f'탑라이트 RED → 구역 {self.current_zone} 미션 완료.'
            )
            self._on_mission_done()
            return

        elapsed = self._now() - self.mission_start_time
        if elapsed >= self.mission_timeout:
            self.get_logger().info(
                f'구역 {self.current_zone} 미션 타임아웃 ({elapsed:.1f}s). 완료 처리.'
            )
            self._on_mission_done()

    def _on_mission_done(self):
        self.state = State.MISSION_DONE
        self.mission_start_time = None
        self._publish_status(f'MISSION_DONE_ZONE_{self.current_zone}')

    # ─────────────────────────────────────────────
    # 공통
    # ─────────────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
