# Internal ROS operation state values
uint8 INIT = 0
uint8 IDLE = 1
uint8 MOVE = 2
uint8 SURVEILLANCE = 3
uint8 DRONE_SURVEILLANCE = 4
uint8 MANUAL_ATTACK = 5
uint8 ASSAULT = 6
uint8 TRACKING = 7
uint8 EMERGENCY_STOP = 8
uint8 ERROR = 9

# App-facing active mode IDs
uint8 ACTIVE_MODE_IDLE = 0
uint8 ACTIVE_MODE_RECON = 1
uint8 ACTIVE_MODE_PROTECT_GENERAL = 2
uint8 ACTIVE_MODE_PROTECT_DRONE = 3
uint8 ACTIVE_MODE_ASSAULT = 6
uint8 ACTIVE_MODE_RETURN_TO_HOME = 7
uint8 ACTIVE_MODE_ESTOP = 8

# Common mission status IDs
uint8 MISSION_NONE = 0
uint8 MISSION_READY = 1
uint8 MISSION_MOVING = 2
uint8 MISSION_PAUSED = 3
uint8 MISSION_REACHED = 4
uint8 MISSION_SURVEILLING = 5
uint8 MISSION_ERROR = 6

uint8 state
uint8 active_mode_id
uint8 mission_status
bool estop_active

bool permission_request_active
float32 crosshair_x
float32 crosshair_y
float32 current_zoom_level

# Robot navigation status
float64 gps_lat
float64 gps_lon
float32 gps_heading
float32 current_speed_mps

# Common mission placeholder fields for app integration
uint16 current_waypoint_index
uint16 total_waypoints
float32 progress_ratio
float32 distance_to_next_wp_m
float32 distance_to_goal_m
uint8 error_code
