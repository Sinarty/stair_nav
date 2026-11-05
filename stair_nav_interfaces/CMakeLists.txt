cmake_minimum_required(VERSION 3.8)
project(stair_nav_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlCommand.msg"
  "msg/MotionStatus.msg"
  "msg/ChannelEvent.msg"
  "msg/ObstacleStatus.msg"
  "msg/SegmentStatus.msg"
  "msg/MissionStatus.msg"
  "msg/MissionEvent.msg"
  "srv/GetNavigationState.srv"
  "action/NavigateSegment.action"
  "action/ExecuteMission.action"
  DEPENDENCIES std_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
