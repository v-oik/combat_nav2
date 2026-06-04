#include <memory>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "robot_localization/srv/from_ll.hpp"
#include "geographic_msgs/msg/geo_point.hpp"

#include "combat_robot_msgs/msg/waypoint.hpp"
#include "combat_robot_msgs/msg/waypoint_list.hpp"
#include "combat_robot_msgs/msg/operation_state.hpp"
#include "combat_robot_msgs/msg/swarm_path_command.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace
{
// SwarmPathCommand command 값
constexpr uint8_t CMD_START = 1;
constexpr uint8_t CMD_STOP = 2;
constexpr uint8_t CMD_PAUSE = 3;
constexpr uint8_t CMD_RESUME = 4;
constexpr uint8_t CMD_LOAD_PATH = 5;

constexpr uint8_t MISSION_ERROR_NONE = 0;
constexpr uint8_t MISSION_ERROR_INVALID_PATH_PAYLOAD = 1;
constexpr uint8_t MISSION_ERROR_PATH_NOT_LOADED = 3;
constexpr uint8_t MISSION_ERROR_INVALID_PATH_COMMAND = 4;

// ----- 태블릿 path_json 최소 파서 -----
// 지원: {"waypoints":[{"lat":..,"lon":..}, ...]}
//       {"coordinates":[[lon,lat], ...]}                (GeoJSON)
//       [{"lat":..,"lon":..}, ...]
std::size_t findMatchingDelimiter(const std::string &t, std::size_t pos, char o, char c)
{
  if (pos >= t.size() || t[pos] != o) return std::string::npos;
  int d = 0;
  for (std::size_t i = pos; i < t.size(); ++i) {
    if (t[i] == o) ++d;
    else if (t[i] == c) { --d; if (d == 0) return i; }
  }
  return std::string::npos;
}

bool extractJsonNumberField(const std::string &obj, const char *name, double *out)
{
  const std::string key = "\"" + std::string(name) + "\"";
  const std::size_t k = obj.find(key);
  if (k == std::string::npos) return false;
  const std::size_t col = obj.find(':', k + key.size());
  if (col == std::string::npos) return false;
  std::size_t s = col + 1;
  while (s < obj.size() && std::isspace(static_cast<unsigned char>(obj[s]))) ++s;
  if (s >= obj.size()) return false;
  char *end = nullptr;
  *out = std::strtod(obj.c_str() + s, &end);
  return end != obj.c_str() + s;
}

bool extractObjectLatLon(const std::string &obj, double *lat, double *lon)
{
  double la = 0.0, lo = 0.0;
  const bool hl = extractJsonNumberField(obj, "lat", &la) ||
                  extractJsonNumberField(obj, "latitude", &la);
  const bool hn = extractJsonNumberField(obj, "lon", &lo) ||
                  extractJsonNumberField(obj, "lng", &lo) ||
                  extractJsonNumberField(obj, "longitude", &lo);
  if (!hl || !hn) return false;
  *lat = la; *lon = lo;
  return true;
}

bool extractNextNumber(const std::string &t, std::size_t *pos, double *out)
{
  std::size_t s = *pos;
  while (s < t.size() && !std::isdigit(static_cast<unsigned char>(t[s])) && t[s] != '-') ++s;
  if (s >= t.size()) return false;
  char *end = nullptr;
  *out = std::strtod(t.c_str() + s, &end);
  if (end == t.c_str() + s) return false;
  *pos = static_cast<std::size_t>(end - t.c_str());
  return true;
}

enum class PathFormat { OBJECT_ARRAY, GEOJSON_COORDINATES };

bool findArrayBounds(const std::string &p, std::size_t *s, std::size_t *e, PathFormat *f)
{
  std::size_t k = p.find("\"waypoints\"");
  if (k != std::string::npos) {
    *s = p.find('[', k); if (*s == std::string::npos) return false;
    *e = findMatchingDelimiter(p, *s, '[', ']'); if (*e == std::string::npos) return false;
    *f = PathFormat::OBJECT_ARRAY; return true;
  }
  k = p.find("\"coordinates\"");
  if (k != std::string::npos) {
    *s = p.find('[', k); if (*s == std::string::npos) return false;
    *e = findMatchingDelimiter(p, *s, '[', ']'); if (*e == std::string::npos) return false;
    *f = PathFormat::GEOJSON_COORDINATES; return true;
  }
  const std::size_t fn = p.find_first_not_of(" \t\r\n");
  if (fn != std::string::npos && p[fn] == '[') {
    *s = fn;
    *e = findMatchingDelimiter(p, *s, '[', ']'); if (*e == std::string::npos) return false;
    *f = PathFormat::OBJECT_ARRAY; return true;
  }
  return false;
}

std::vector<std::pair<double, double>> parsePathJson(const std::string &payload)
{
  std::vector<std::pair<double, double>> out;
  std::size_t a_s = 0, a_e = 0;
  PathFormat fmt = PathFormat::OBJECT_ARRAY;
  if (!findArrayBounds(payload, &a_s, &a_e, &fmt)) return out;

  std::size_t pos = a_s + 1;
  while (pos < a_e) {
    double lat = 0.0, lon = 0.0;
    bool ok = false;
    if (fmt == PathFormat::OBJECT_ARRAY) {
      const std::size_t os = payload.find('{', pos);
      if (os == std::string::npos || os > a_e) break;
      const std::size_t oe = findMatchingDelimiter(payload, os, '{', '}');
      if (oe == std::string::npos || oe > a_e) break;
      ok = extractObjectLatLon(payload.substr(os, oe - os + 1), &lat, &lon);
      pos = oe + 1;
    } else {
      const std::size_t cs = payload.find('[', pos);
      if (cs == std::string::npos || cs > a_e) break;
      const std::size_t ce = findMatchingDelimiter(payload, cs, '[', ']');
      if (ce == std::string::npos || ce > a_e) break;
      const std::string coord = payload.substr(cs, ce - cs + 1);
      std::size_t inner = 0;
      double parsed_lon = 0.0, parsed_lat = 0.0;
      if (extractNextNumber(coord, &inner, &parsed_lon) &&
          extractNextNumber(coord, &inner, &parsed_lat))
      {
        lat = parsed_lat; lon = parsed_lon; ok = true;
      }
      pos = ce + 1;
    }
    if (ok) out.emplace_back(lat, lon);
  }
  return out;
}

}  // namespace


class GpsMissionControlNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GpsMissionControlNode() : Node("gps_mission_control_node")
    {
        // 태블릿 → robot_server → /swarm/path_command
        path_cmd_sub_ = this->create_subscription<combat_robot_msgs::msg::SwarmPathCommand>(
            "/swarm/path_command", 10,
            std::bind(&GpsMissionControlNode::path_command_callback, this, _1));

        // 내부/테스트용 직접 입력 (way_test.py 등). 필요 없으면 통째로 제거 가능.
        mission_sub_ = this->create_subscription<combat_robot_msgs::msg::WaypointList>(
            "/mission_input", 10,
            std::bind(&GpsMissionControlNode::mission_callback, this, _1));

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        from_ll_client_ = this->create_client<robot_localization::srv::FromLL>("/fromLL");
        mission_state_pub_ = this->create_publisher<combat_robot_msgs::msg::OperationState>(
            "/swarm/mission_state", 10);
        status_timer_ = this->create_wall_timer(
            500ms, std::bind(&GpsMissionControlNode::publish_mission_status, this));

        // 태블릿 위치표시용: 로봇 GPS 위경도/heading/속도 캐싱 → OperationState 에 채워 발행.
        fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", 10,
            [this](sensor_msgs::msg::NavSatFix::SharedPtr m){
                gps_lat_ = m->latitude; gps_lon_ = m->longitude; });
        heading_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/edge_heading", 10,   // gnss_heading 의 컴퍼스 heading(deg)
            [this](std_msgs::msg::Float64::SharedPtr m){
                gps_heading_ = static_cast<float>(m->data); });
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr m){
                current_speed_ = static_cast<float>(std::hypot(
                    m->twist.twist.linear.x, m->twist.twist.linear.y)); });

        RCLCPP_INFO(this->get_logger(),
                    "GPS Mission Control 준비 — 입력: /swarm/path_command, /mission_input");
    }

private:
    // ---- 상태 ----
    std::vector<std::pair<double, double>> active_points_;   // (lat, lon) iterating
    std::size_t active_index_ = 0;
    bool active_ = false;
    bool paused_ = false;   // PAUSE 로 현재 goal 을 cancel 한 상태 (active_index_ 유지)

    // 태블릿 LOAD_PATH로 받아둔 경로 (START 전까지 대기)
    std::vector<std::pair<double, double>> pending_path_;

    GoalHandleNav::SharedPtr active_goal_;
    uint8_t mission_status_ = combat_robot_msgs::msg::OperationState::MISSION_NONE;
    uint8_t mission_error_code_ = MISSION_ERROR_NONE;
    float distance_to_next_wp_m_ = 0.0f;
    float distance_to_goal_m_ = 0.0f;

    rclcpp::Subscription<combat_robot_msgs::msg::SwarmPathCommand>::SharedPtr path_cmd_sub_;
    rclcpp::Subscription<combat_robot_msgs::msg::WaypointList>::SharedPtr mission_sub_;
    rclcpp::Publisher<combat_robot_msgs::msg::OperationState>::SharedPtr mission_state_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr from_ll_client_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    double gps_lat_ = 0.0;
    double gps_lon_ = 0.0;
    float gps_heading_ = 0.0f;
    float current_speed_ = 0.0f;

    std::size_t current_waypoint_count() const
    {
        return active_ ? active_points_.size() : pending_path_.size();
    }

    uint16_t clamp_waypoint_count(std::size_t value) const
    {
        constexpr std::size_t max_u16 = 65535u;
        return static_cast<uint16_t>(std::min(value, max_u16));
    }

    float mission_progress_ratio() const
    {
        const std::size_t total = current_waypoint_count();
        if (total == 0) {
            return 0.0f;
        }
        const std::size_t completed = std::min(active_index_, total);
        return static_cast<float>(completed) / static_cast<float>(total);
    }

    // robot_server gates tablet mode changes on operation_state == IDLE. Report the
    // real state derived from the mission lifecycle: IDLE while idle/ready (so the
    // operator can start a mode), MOVE while a path is executing/paused, ERROR on fault.
    static uint8_t operation_state_from_mission(uint8_t mission_status)
    {
        using OperationState = combat_robot_msgs::msg::OperationState;
        switch (mission_status) {
            case OperationState::MISSION_MOVING:
            case OperationState::MISSION_PAUSED:
            case OperationState::MISSION_REACHED:
            case OperationState::MISSION_SURVEILLING:
                return OperationState::MOVE;
            case OperationState::MISSION_ERROR:
                return OperationState::ERROR;
            case OperationState::MISSION_NONE:
            case OperationState::MISSION_READY:
            default:
                return OperationState::IDLE;
        }
    }

    void publish_mission_status()
    {
        if (!mission_state_pub_) {
            return;
        }

        combat_robot_msgs::msg::OperationState msg;
        msg.state = operation_state_from_mission(mission_status_);
        // mission_control does not own the operation mode (RECON/PROTECT/...);
        // robot_server is the authority for active_mode_id, so report neutral here.
        msg.active_mode_id = combat_robot_msgs::msg::OperationState::ACTIVE_MODE_IDLE;
        msg.mission_status = mission_status_;
        msg.current_waypoint_index = clamp_waypoint_count(active_index_);
        msg.total_waypoints = clamp_waypoint_count(current_waypoint_count());
        msg.progress_ratio = mission_progress_ratio();
        msg.distance_to_next_wp_m = distance_to_next_wp_m_;
        msg.distance_to_goal_m = distance_to_goal_m_;
        msg.gps_lat = gps_lat_;
        msg.gps_lon = gps_lon_;
        msg.gps_heading = gps_heading_;
        msg.current_speed_mps = current_speed_;
        msg.error_code = mission_error_code_;
        mission_state_pub_->publish(msg);
    }

    void update_mission_status(uint8_t status, uint8_t error_code = MISSION_ERROR_NONE)
    {
        mission_status_ = status;
        mission_error_code_ = error_code;
        publish_mission_status();
    }

    // ---------------- 태블릿 SwarmPathCommand ----------------
    void path_command_callback(const combat_robot_msgs::msg::SwarmPathCommand::SharedPtr msg)
    {
        switch (msg->command) {
            case CMD_LOAD_PATH: {
                if (msg->path_json.empty()) {
                    RCLCPP_WARN(this->get_logger(), "LOAD_PATH payload 비어있음");
                    update_mission_status(
                        combat_robot_msgs::msg::OperationState::MISSION_ERROR,
                        MISSION_ERROR_INVALID_PATH_PAYLOAD);
                    return;
                }
                auto pts = parsePathJson(msg->path_json);
                if (pts.empty()) {
                    RCLCPP_WARN(this->get_logger(), "LOAD_PATH path_json 에서 waypoint 추출 실패");
                    update_mission_status(
                        combat_robot_msgs::msg::OperationState::MISSION_ERROR,
                        MISSION_ERROR_INVALID_PATH_PAYLOAD);
                    return;
                }
                pending_path_ = std::move(pts);
                if (!active_) {
                    active_index_ = 0;
                    distance_to_next_wp_m_ = 0.0f;
                    distance_to_goal_m_ = 0.0f;
                    update_mission_status(
                        combat_robot_msgs::msg::OperationState::MISSION_READY);
                } else {
                    publish_mission_status();
                }
                RCLCPP_INFO(this->get_logger(),
                            "[Tablet] LOAD_PATH 캐싱: %zu wps", pending_path_.size());
                break;
            }
            case CMD_START:
                if (pending_path_.empty()) {
                    RCLCPP_WARN(this->get_logger(), "[Tablet] START 받았으나 경로 없음");
                    update_mission_status(
                        combat_robot_msgs::msg::OperationState::MISSION_ERROR,
                        MISSION_ERROR_PATH_NOT_LOADED);
                    return;
                }
                start_mission(pending_path_);
                break;
            case CMD_STOP:
                pending_path_.clear();
                cancel_mission();
                RCLCPP_INFO(this->get_logger(), "[Tablet] STOP");
                break;
            case CMD_PAUSE:
                if (!active_) {
                    RCLCPP_WARN(this->get_logger(), "[Tablet] PAUSE 받았으나 주행 중 아님");
                    return;
                }
                if (paused_) {
                    RCLCPP_WARN(this->get_logger(), "[Tablet] PAUSE 받았으나 이미 일시정지 상태");
                    return;
                }
                paused_ = true;
                // 현재 goal 을 취소 (active_index_ 는 유지 → RESUME 시 동일 wp 부터 재개).
                // goal 이 아직 accept 전이면 goal_response_callback 에서 즉시 cancel 처리됨.
                if (active_goal_) {
                    nav_client_->async_cancel_goal(active_goal_);
                }
                update_mission_status(
                    combat_robot_msgs::msg::OperationState::MISSION_PAUSED);
                RCLCPP_INFO(this->get_logger(),
                            "[Tablet] PAUSE — wp[%zu/%zu] 에서 정지",
                            active_index_ + 1, active_points_.size());
                break;
            case CMD_RESUME:
                if (!active_) {
                    RCLCPP_WARN(this->get_logger(), "[Tablet] RESUME 받았으나 미션 없음");
                    return;
                }
                if (!paused_) {
                    RCLCPP_WARN(this->get_logger(), "[Tablet] RESUME 받았으나 일시정지 상태 아님");
                    return;
                }
                paused_ = false;
                update_mission_status(
                    combat_robot_msgs::msg::OperationState::MISSION_MOVING);
                RCLCPP_INFO(this->get_logger(),
                            "[Tablet] RESUME — wp[%zu/%zu] 부터 재개",
                            active_index_ + 1, active_points_.size());
                execute_next_waypoint();
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "unknown path command: %u", msg->command);
                update_mission_status(
                    combat_robot_msgs::msg::OperationState::MISSION_ERROR,
                    MISSION_ERROR_INVALID_PATH_COMMAND);
        }
    }

    // ---------------- /mission_input 직접 입력 ----------------
    void mission_callback(const combat_robot_msgs::msg::WaypointList::SharedPtr msg)
    {
        if (msg->waypoints.empty()) {
            RCLCPP_WARN(this->get_logger(), "/mission_input: 빈 waypoint");
            return;
        }
        std::vector<std::pair<double, double>> pts;
        pts.reserve(msg->waypoints.size());
        for (const auto &wp : msg->waypoints) {
            pts.emplace_back(wp.way_lat, wp.way_lon);
        }
        RCLCPP_INFO(this->get_logger(),
                    "/mission_input 수신: mission_id=%d, %zu wps",
                    msg->mission_id, pts.size());
        start_mission(pts);
    }

    // ---------------- 공통 진입점 ----------------
    void start_mission(const std::vector<std::pair<double, double>> &pts)
    {
        if (active_) {
            RCLCPP_WARN(this->get_logger(), "이미 미션 실행 중 — 새 요청 거절");
            return;
        }
        if (!nav_client_->wait_for_action_server(3s)) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 navigate_to_pose 서버 연결 실패");
            update_mission_status(
                combat_robot_msgs::msg::OperationState::MISSION_ERROR);
            return;
        }
        active_points_ = pts;
        active_index_ = 0;
        active_ = true;
        paused_ = false;
        distance_to_next_wp_m_ = 0.0f;
        distance_to_goal_m_ = 0.0f;
        update_mission_status(
            combat_robot_msgs::msg::OperationState::MISSION_MOVING);
        execute_next_waypoint();
    }

    void cancel_mission()
    {
        if (active_ && active_goal_) {
            nav_client_->async_cancel_goal(active_goal_);
        }
        active_ = false;
        paused_ = false;
        active_goal_.reset();
        active_points_.clear();
        active_index_ = 0;
        distance_to_next_wp_m_ = 0.0f;
        distance_to_goal_m_ = 0.0f;
        update_mission_status(
            combat_robot_msgs::msg::OperationState::MISSION_NONE);
    }

    void execute_next_waypoint()
    {
        if (active_index_ >= active_points_.size()) {
            RCLCPP_INFO(this->get_logger(), "모든 waypoint 완료 (%zu)", active_points_.size());
            active_ = false;
            distance_to_next_wp_m_ = 0.0f;
            distance_to_goal_m_ = 0.0f;
            update_mission_status(
                combat_robot_msgs::msg::OperationState::MISSION_REACHED);
            return;
        }
        if (!from_ll_client_->wait_for_service(3s)) {
            RCLCPP_ERROR(this->get_logger(),
                         "/fromLL 서비스 없음 (navsat_transform_node 확인)");
            active_ = false;
            distance_to_next_wp_m_ = 0.0f;
            distance_to_goal_m_ = 0.0f;
            update_mission_status(
                combat_robot_msgs::msg::OperationState::MISSION_ERROR);
            return;
        }
        const auto [lat, lon] = active_points_[active_index_];
        auto req = std::make_shared<robot_localization::srv::FromLL::Request>();
        req->ll_point.latitude = lat;
        req->ll_point.longitude = lon;
        req->ll_point.altitude = 0.0;
        from_ll_client_->async_send_request(req,
            std::bind(&GpsMissionControlNode::from_ll_response_callback, this, _1));
    }

    void from_ll_response_callback(rclcpp::Client<robot_localization::srv::FromLL>::SharedFuture future)
    {
        if (!active_) {
            RCLCPP_INFO(this->get_logger(),
                        "STOP 이후 늦은 /fromLL 응답 도착 — 무시");
            return;
        }
        if (paused_) {
            // PAUSE 가 /fromLL 대기 중 도착 — goal 전송 보류. RESUME 시 재요청됨.
            RCLCPP_INFO(this->get_logger(),
                        "PAUSE 중 /fromLL 응답 도착 — goal 전송 보류 (RESUME 대기)");
            return;
        }
        auto response = future.get();
        const double map_x = response->map_point.x;
        const double map_y = response->map_point.y;

        RCLCPP_INFO(this->get_logger(),
                    "wp[%zu/%zu] (lat=%.7f, lon=%.7f) → map(%.2f, %.2f) 주행",
                    active_index_ + 1, active_points_.size(),
                    active_points_[active_index_].first,
                    active_points_[active_index_].second,
                    map_x, map_y);

        auto goal = NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->now();
        goal.pose.pose.position.x = map_x;
        goal.pose.pose.position.y = map_y;
        goal.pose.pose.orientation.w = 1.0;

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        opts.goal_response_callback =
            std::bind(&GpsMissionControlNode::goal_response_callback, this, _1);
        opts.feedback_callback =
            std::bind(&GpsMissionControlNode::feedback_callback, this, _1, _2);
        opts.result_callback =
            std::bind(&GpsMissionControlNode::result_callback, this, _1);

        nav_client_->async_send_goal(goal, opts);
    }

    void feedback_callback(
        GoalHandleNav::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        if (!active_ || paused_ || !feedback) {
            return;
        }
        if (std::isfinite(feedback->distance_remaining)) {
            const float remaining = std::max(0.0f, static_cast<float>(feedback->distance_remaining));
            distance_to_next_wp_m_ = remaining;
            distance_to_goal_m_ = remaining;
        }
        publish_mission_status();
    }

    void goal_response_callback(const GoalHandleNav::SharedPtr &gh)
    {
        if (!gh) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 가 goal 을 거절함");
            active_ = false;
            active_goal_.reset();
            update_mission_status(
                combat_robot_msgs::msg::OperationState::MISSION_ERROR);
            return;
        }
        if (!active_) {
            // STOP 이 goal 전송과 accept 사이에 도착 — 즉시 cancel
            RCLCPP_INFO(this->get_logger(),
                        "STOP 이후 goal 이 accept 됨 — 즉시 cancel");
            nav_client_->async_cancel_goal(gh);
            return;
        }
        if (paused_) {
            // PAUSE 가 goal 전송과 accept 사이에 도착 — 즉시 cancel (active_index_ 유지)
            RCLCPP_INFO(this->get_logger(),
                        "PAUSE 중 goal 이 accept 됨 — 즉시 cancel");
            nav_client_->async_cancel_goal(gh);
            return;
        }
        active_goal_ = gh;
        update_mission_status(
            combat_robot_msgs::msg::OperationState::MISSION_MOVING);
    }

    void result_callback(const GoalHandleNav::WrappedResult &result)
    {
        active_goal_.reset();
        if (!active_) {
            // STOP 으로 이미 종료된 상태에서 늦게 도착한 result — 무시
            RCLCPP_INFO(this->get_logger(),
                        "STOP 이후 늦은 nav result 도착 (code=%d) — 무시",
                        static_cast<int>(result.code));
            return;
        }
        if (paused_) {
            // PAUSE 로 인한 cancel 결과 — 미션 실패로 처리하지 않음. active_index_ 유지.
            RCLCPP_INFO(this->get_logger(),
                        "PAUSE 로 wp[%zu] goal cancel 됨 (code=%d) — RESUME 대기",
                        active_index_ + 1, static_cast<int>(result.code));
            update_mission_status(
                combat_robot_msgs::msg::OperationState::MISSION_PAUSED);
            return;
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "wp[%zu] 도착", active_index_ + 1);
            ++active_index_;
            distance_to_next_wp_m_ = 0.0f;
            distance_to_goal_m_ = 0.0f;
            update_mission_status(
                combat_robot_msgs::msg::OperationState::MISSION_MOVING);
            execute_next_waypoint();
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "wp[%zu] 실패/취소 (code=%d) — 미션 중단",
                         active_index_ + 1, static_cast<int>(result.code));
            active_ = false;
            distance_to_next_wp_m_ = 0.0f;
            distance_to_goal_m_ = 0.0f;
            update_mission_status(
                combat_robot_msgs::msg::OperationState::MISSION_ERROR);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsMissionControlNode>());
    rclcpp::shutdown();
    return 0;
}
