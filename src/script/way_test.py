#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL
from geographic_msgs.msg import GeoPoint

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('gps_waypoint_sender_fromll')

    # navsat_transform_node가 제공하는 변환 서비스 클라이언트
    from_ll_cli = node.create_client(FromLL, '/fromLL')

    nav_client = ActionClient(node, NavigateThroughPoses, 'navigate_through_poses')

    node.get_logger().info('/fromLL 서비스 대기중... (navsat_transform_node가 켜져 있어야 합니다)')
    while not from_ll_cli.wait_for_service(timeout_sec=1.0):
        pass

    #  GPS way point

    gps_targets = [
        # example
        (36.610149526, 127.28768799),
        (36.61011546998354, 127.28699941652863), 
        (36.6101105350, 127.286555705)
    ]

    map_poses = []
    
    # 각각의 GPS 좌표를 /fromLL 서비스에 보내서 정확한 Map(X, Y) 좌표로 변환받음
    for lat, lon in gps_targets:
        req = FromLL.Request()
        req.ll_point.latitude = lat
        req.ll_point.longitude = lon
        req.ll_point.altitude = 0.0

      
        future = from_ll_cli.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        resp = future.result()

        # 변환된 X, Y 좌표를 Nav2 웨이포인트 형식으로 포장
        wp = PoseStamped()
        wp.header.frame_id = 'map'
        wp.pose.position.x = resp.map_point.x
        wp.pose.position.y = resp.map_point.y
        wp.pose.orientation.w = 1.0 
        map_poses.append(wp)
        
        node.get_logger().info(f'GPS 변환 성공: ({lat:.6f}, {lon:.6f}) -> Map XY({resp.map_point.x:.2f}m, {resp.map_point.y:.2f}m)')

    # 변환이 끝났으니 Nav2 서버에 주행 명령 전송
    node.get_logger().info('Nav2 주행 서버 대기중...')
    nav_client.wait_for_server()

    goal_msg = NavigateThroughPoses.Goal()
    goal_msg.poses = map_poses

    node.get_logger().info('변환된 웨이포인트로 주행 명령을 전송합니다!')
    future = nav_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)

    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().error('거절.')
    else:
        node.get_logger().info('승인.')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        node.get_logger().info('주행 완료')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()