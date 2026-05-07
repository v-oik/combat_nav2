import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math

class GpsNavMaster(Node):
    def __init__(self):
        super().__init__('gps_nav_master')
        
        # 1. 기준점 (ekf.yaml의 datum과 반드시 일치)
        self.datum_lat = 37.5932637413
        self.datum_lon = 126.6233586871
        
        # 2. 목표 GPS 좌표 (이미지 좌표)
        self.target_lat = 37.594135
        self.target_lon = 126.623423
        
        # 3. Nav2 네비게이터 초기화
        # 💡 중요: GPS 환경이므로 AMCL 대신 bt_navigator를 체크하도록 설정
        self.nav = BasicNavigator()
        
        self.EARTH_RADIUS = 6378137.0 

        # 로봇 준비 대기
        self.wait_for_nav2()

    def wait_for_nav2(self):
        self.get_logger().info('Nav2 활성화 대기 중 (GPS Mode)...')
        
        # 💡 수정: AMCL 서비스가 없어도 넘어가도록 서버 이름을 지정합니다.
        # amcl 노드가 아닌 bt_navigator가 올라왔는지 확인합니다.
        self.nav.waitUntilNav2Active(localizer='bt_navigator')
        
        self.get_logger().info('Nav2 준비 완료! 주행을 시작합니다.')
        self.start_navigation()

    def lat_lon_to_map_xy(self, lat, lon):
        d_lat = math.radians(lat - self.datum_lat)
        d_lon = math.radians(lon - self.datum_lon)
        y = d_lat * self.EARTH_RADIUS
        x = d_lon * self.EARTH_RADIUS * math.cos(math.radians(self.datum_lat))
        return x, y

    def start_navigation(self):
        tx, ty = self.lat_lon_to_map_xy(self.target_lat, self.target_lon)
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = tx
        goal_pose.pose.position.y = ty
        goal_pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'목표지로 이동 명령: X={tx:.2f}m, Y={ty:.2f}m')
        
        # 주행 시작
        self.nav.goToPose(goal_pose)

        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            if feedback:
                # distance_remaining 피드백 출력
                print(f'남은 거리: {feedback.distance_remaining:.2f} m', end='\r')

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('도착 완료!')
        else:
            self.get_logger().error('주행 실패 또는 취소')

def main():
    rclpy.init()
    node = GpsNavMaster()
    rclpy.shutdown()

if __name__ == '__main__':
    main()