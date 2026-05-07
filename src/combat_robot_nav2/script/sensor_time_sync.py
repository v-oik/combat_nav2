#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, NavSatFix, Imu

class SensorTimeSyncNode(Node):
    def __init__(self):
        super().__init__('sensor_time_sync_node')
        
        # 1. LiDAR 동기화
        self.lidar_sub = self.create_subscription(PointCloud2, '/rslidar_points_raw', self.lidar_cb, 10)
        self.lidar_pub = self.create_publisher(PointCloud2, '/front_lidar/pointcloud', 10)

        self.get_logger().info("센서 시간 동기화 노드가 시작되었습니다. (LiDAR, GNSS, IMU)")

    def lidar_cb(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'front_lidar_link'
        self.lidar_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorTimeSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()