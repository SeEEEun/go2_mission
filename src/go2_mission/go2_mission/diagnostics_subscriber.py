import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray


STATUS_LEVEL = {0: 'OK', 1: 'WARN', 2: 'ERROR'}

DEVICES = {'device1', 'device2', 'device3', 'device4'}


class DiagnosticsSubscriber(Node):
    def __init__(self):
        super().__init__('diagnostics_subscriber')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/factory/diagnostics',
            self._callback,
            qos,
        )

        # 설비별 최신 상태 저장
        self.device_status: dict[str, dict] = {}

        self.get_logger().info('DiagnosticsSubscriber 시작. /factory/diagnostics 수신 대기 중...')

    def _callback(self, msg: DiagnosticArray):
        stamp = msg.header.stamp
        timestamp = f'{stamp.sec}.{stamp.nanosec:09d}'

        for status in msg.status:
            name = status.name
            if name not in DEVICES:
                self.get_logger().warn(f'알 수 없는 설비 이름: {name}')
                continue

            level_str = STATUS_LEVEL.get(status.level, f'UNKNOWN({status.level})')
            values = {kv.key: kv.value for kv in status.values}

            self.device_status[name] = {
                'level': status.level,
                'level_str': level_str,
                'hardware_id': status.hardware_id,
                'message': status.message,
                'values': values,
                'timestamp': timestamp,
            }

            self.get_logger().info(
                f'[{name}] level={level_str} | hw_id={status.hardware_id} '
                f'| msg="{status.message}" | values={values} | t={timestamp}'
            )

            if status.level == 2:
                self.get_logger().error(f'[{name}] ERROR 발생! 즉시 확인 필요.')
            elif status.level == 1:
                self.get_logger().warn(f'[{name}] WARN 상태. 예방 정비 필요.')

    def get_device_status(self, device_name: str) -> dict | None:
        return self.device_status.get(device_name)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
