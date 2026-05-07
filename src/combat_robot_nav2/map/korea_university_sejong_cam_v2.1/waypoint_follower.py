import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import csv
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('gps_waypoint_follower')

        # 🚀 본인의 절대 경로에 맞게 확인하세요.
        self.csv_path = '/home/skyautonet/birdro_nav2/src/bird_nav2/map/korea_university_sejong_cam_v2.1/waypoints.csv'
        self.arrival_distance = 1.0  # 1m 반경 이내면 도착으로 인정하고 부드럽게 통과

        self.waypoints = self.load_waypoints(self.csv_path)
        self.current_idx = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.is_loc_received = False

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/global', self.odom_callback, 10)

        self.timer = self.create_timer(1.0, self.control_loop)
        self.get_logger().info(f"✅ 전술로봇 웨이포인트 주행 준비: {len(self.waypoints)}개 로드 완료.")

    def load_waypoints(self, file_path):
        wps = []
        try:
            with open(file_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    wps.append((float(row['x']), float(row['y'])))
            return wps
        except Exception as e:
            self.get_logger().error(f"❌ CSV 로드 실패: {e}")
            return []

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.is_loc_received = True

    def control_loop(self):
        if not self.is_loc_received or self.current_idx >= len(self.waypoints):
            return

        target_x, target_y = self.waypoints[self.current_idx]
        dist = math.sqrt((target_x - self.robot_x)**2 + (target_y - self.robot_y)**2)

        if dist < self.arrival_distance:
            self.get_logger().info(f"📍 포인트 {self.current_idx + 1}/{len(self.waypoints)} 도달! 다음 목표로 이동합니다.")
            self.current_idx += 1
            return

        self.publish_goal(target_x, target_y)

    def publish_goal(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        
        # 🚀 수정: 차량이 웨이포인트를 향하는 각도를 수학적으로 계산하여 훅(비틀림) 방지
        yaw = math.atan2(y - self.robot_y, x - self.robot_x)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.goal_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()