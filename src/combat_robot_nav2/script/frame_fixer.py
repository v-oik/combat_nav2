#!/usr/bin/env python3
"""
/odometry/gps의 frame_id를 'map'으로 바꿔서 /odometry/gps_map으로 재발행.
navsat_transform이 잘못 설정하는 frame_id를 우회.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class FrameFixer(Node):
    def __init__(self):
        super().__init__('frame_fixer')

        self.declare_parameter('input_topic', '/odometry/gps')
        self.declare_parameter('output_topic', '/odometry/gps_map')
        self.declare_parameter('target_frame_id', 'map')

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame_id').value

        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.sub = self.create_subscription(Odometry, in_topic, self.cb, 10)

        self._msg_count = 0
        self.create_timer(5.0, self._status_timer)

        self.get_logger().info(
            f'✅ frame_fixer: {in_topic} → {out_topic} '
            f'(frame_id forced to "{self.target_frame}")'
        )

    def _status_timer(self):
        self.get_logger().info(f'[frame_fixer] relayed {self._msg_count} msgs in 5s')
        self._msg_count = 0

    def cb(self, msg):
        msg.header.frame_id = self.target_frame
        self.pub.publish(msg)
        self._msg_count += 1


def main():
    rclpy.init()
    node = FrameFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()