std_msgs/Header header

uint8 CMD_NONE = 0
uint8 CMD_START = 1
uint8 CMD_STOP = 2
uint8 CMD_PAUSE = 3
uint8 CMD_RESUME = 4
uint8 CMD_LOAD_PATH = 5
uint8 CMD_COMPLETE = 6  # path follower / operator signals 'waypoints done'; FSM switches PROTECT modes from drive to engage

uint8 command
uint16 num_waypoints
string path_json
