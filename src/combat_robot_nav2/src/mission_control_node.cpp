#include <memory>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

// GPS 변환 공식 서비스 및 메시지 헤더 (robot_localization, geographic_msgs)
#include "robot_localization/srv/from_ll.hpp"
#include "geographic_msgs/msg/geo_point.hpp"

// 커스텀 메시지 헤더
#include "combat_robot_msgs/msg/waypoint.hpp"
#include "combat_robot_msgs/msg/waypoint_list.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class GpsMissionControlNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GpsMissionControlNode() : Node("gps_mission_control_node"),
                              mission_active_(false), current_wp_index_(0)
    {
        // 상태 코드 정의
        STATUS_WAITING = 0;
        STATUS_RUNNING = 1;
        STATUS_SUCCESS = 2;
        STATUS_REJECTED = 3;
        STATUS_FAILED = -1;

        // Subscriber & Publisher 설정
        mission_sub_ = this->create_subscription<combat_robot_msgs::msg::WaypointList>(
            "/mission_input", 10, std::bind(&GpsMissionControlNode::mission_callback, this, _1));

        wp_status_pub_ = this->create_publisher<combat_robot_msgs::msg::Waypoint>("/waypoint_status", 10);
        mission_status_pub_ = this->create_publisher<combat_robot_msgs::msg::WaypointList>("/mission_status", 10);

        // Nav2 Action Client (NavigateToPose)
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // GPS -> Map 변환 Service Client (navsat_transform_node의 서비스)
        from_ll_client_ = this->create_client<robot_localization::srv::FromLL>("/fromLL");

        RCLCPP_INFO(this->get_logger(), "GPS Mission Control Node 준비 완료 (nav2_gps_waypoint 기반 변환 사용).");
    }

private:
    int STATUS_WAITING;
    int STATUS_RUNNING;
    int STATUS_SUCCESS;
    int STATUS_REJECTED;
    int STATUS_FAILED;

    bool mission_active_;
    combat_robot_msgs::msg::WaypointList current_mission_;
    size_t current_wp_index_;

    rclcpp::Subscription<combat_robot_msgs::msg::WaypointList>::SharedPtr mission_sub_;
    rclcpp::Publisher<combat_robot_msgs::msg::Waypoint>::SharedPtr wp_status_pub_;
    rclcpp::Publisher<combat_robot_msgs::msg::WaypointList>::SharedPtr mission_status_pub_;
    
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr from_ll_client_;

    // 외부 미션 수신 콜백
    void mission_callback(const combat_robot_msgs::msg::WaypointList::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "새로운 GPS 미션 수신: Mission ID [%d]", msg->mission_id);

        if (mission_active_)
        {
            RCLCPP_WARN(this->get_logger(), "미션 실행 중입니다. 새 미션을 거절합니다.");
            publish_mission_status(msg->mission_id, msg->mode, STATUS_REJECTED, msg->waypoints);
            return;
        }

        if (msg->waypoints.empty())
        {
            RCLCPP_WARN(this->get_logger(), "웨이포인트가 비어 있습니다.");
            publish_mission_status(msg->mission_id, msg->mode, STATUS_REJECTED, msg->waypoints);
            return;
        }

        if (!nav_client_->wait_for_action_server(3s))
        {
            RCLCPP_ERROR(this->get_logger(), "Nav2 Action Server 연결 실패");
            publish_mission_status(msg->mission_id, msg->mode, STATUS_FAILED, msg->waypoints);
            return;
        }

        current_mission_ = *msg;
        current_wp_index_ = 0;
        mission_active_ = true;

        publish_mission_status(current_mission_.mission_id, current_mission_.mode, STATUS_RUNNING, current_mission_.waypoints);

        // 첫 번째 웨이포인트 진행
        execute_next_waypoint();
    }

    // 다음 GPS 웨이포인트를 Map 좌표로 변환 요청
    void execute_next_waypoint()
    {
        if (current_wp_index_ >= current_mission_.waypoints.size())
        {
            RCLCPP_INFO(this->get_logger(), "Mission ID [%d] 모든 웨이포인트 완료!", current_mission_.mission_id);
            publish_mission_status(current_mission_.mission_id, current_mission_.mode, STATUS_SUCCESS, current_mission_.waypoints);
            mission_active_ = false;
            return;
        }

        auto wp = current_mission_.waypoints[current_wp_index_];

        if (!from_ll_client_->wait_for_service(3s))
        {
            RCLCPP_ERROR(this->get_logger(), "/fromLL 서비스가 활성화되지 않았습니다. (navsat_transform_node 확인 필요)");
            handle_waypoint_failure();
            return;
        }

        // GPS 위도/경도를 Map 좌표로 변환해달라고 서비스 요청
        auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
        request->ll_point.latitude = wp.way_lat;
        request->ll_point.longitude = wp.way_lon;
        request->ll_point.altitude = 0.0;

        from_ll_client_->async_send_request(request, 
            std::bind(&GpsMissionControlNode::from_ll_response_callback, this, _1));
    }

    // 좌표 변환 완료 후 Nav2로 주행 명령 전송
    void from_ll_response_callback(rclcpp::Client<robot_localization::srv::FromLL>::SharedFuture future)
    {
        auto response = future.get();
        double map_x = response->map_point.x;
        double map_y = response->map_point.y;

        auto wp = current_mission_.waypoints[current_wp_index_];
        RCLCPP_INFO(this->get_logger(), "Way ID: %d [GPS 변환 완료 -> Map X: %.2f, Y: %.2f] 주행 시작", wp.way_id, map_x, map_y);

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = map_x;
        goal_msg.pose.pose.position.y = map_y;
        goal_msg.pose.pose.orientation.w = 1.0; 

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&GpsMissionControlNode::goal_response_callback, this, _1);
        send_goal_options.result_callback = std::bind(&GpsMissionControlNode::result_callback, this, _1);

        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleNav::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "목표가 Nav2 서버에 의해 거절되었습니다.");
            handle_waypoint_failure();
        }
    }

    void result_callback(const GoalHandleNav::WrappedResult &result)
    {
        auto wp = current_mission_.waypoints[current_wp_index_];

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Way ID: %d 도착 성공", wp.way_id);
            publish_waypoint_status(wp.way_id, STATUS_SUCCESS);

            current_wp_index_++;
            execute_next_waypoint();
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Way ID: %d 주행 실패/취소", wp.way_id);
            handle_waypoint_failure();
            break;
        }
    }

    void handle_waypoint_failure()
    {
        auto wp = current_mission_.waypoints[current_wp_index_];
        publish_waypoint_status(wp.way_id, STATUS_FAILED);
        publish_mission_status(current_mission_.mission_id, current_mission_.mode, STATUS_FAILED, current_mission_.waypoints);
        mission_active_ = false;
    }

    void publish_waypoint_status(int w_id, int status)
    {
        combat_robot_msgs::msg::Waypoint msg;
        msg.way_id = w_id;
        msg.way_status = status;
        wp_status_pub_->publish(msg);
    }

    void publish_mission_status(int m_id, int mode, int status, const std::vector<combat_robot_msgs::msg::Waypoint> &waypoints)
    {
        combat_robot_msgs::msg::WaypointList msg;
        msg.mission_id = m_id;
        msg.mode = mode;
        msg.mission_status = status;
        msg.waypoints = waypoints;
        mission_status_pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpsMissionControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}