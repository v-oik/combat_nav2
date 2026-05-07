cmake_minimum_required(VERSION 3.8)
project(combat_robot_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 기본 ament 빌드 툴
find_package(ament_cmake REQUIRED)

# 2. C++ 미션 컨트롤 노드용 추가 의존성
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(combat_robot_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(geographic_msgs REQUIRED)

# 3. C++ 소스 코드 컴파일 (src/mission_control_node.cpp를 빌드)
add_executable(mission_control_node src/mission_control_node.cpp)

# 4. 컴파일된 노드에 의존성 연결
ament_target_dependencies(mission_control_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  combat_robot_msgs
  robot_localization
  geographic_msgs
)

# 5. 빌드된 실행 파일 설치 (ros2 run으로 실행할 수 있도록 lib 경로에 설치)
install(TARGETS
  mission_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# 6. 기존 설정 파일 및 디렉토리 설치 (src는 컴파일되므로 제외)
install(DIRECTORY
  launch
  config
  map
  rviz
  world
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  script/sensor_time_sync.py
  script/can_reader.py
  script/gnss_heading.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()