cmake_minimum_required(VERSION 3.8)
project(robotic_arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectInfo.msg"
  "msg/DetectedObject.msg"
  "msg/DetectedObjects.msg"
  "srv/ListObjects.srv"
  "srv/MoveObject.srv"
  "action/Pick.action"
  "action/Place.action"
  "action/MoveTo.action"
  "action/Home.action"
  "action/Reset.action"
  "action/OpenGripper.action"
  "action/CloseGripper.action"
  DEPENDENCIES std_msgs
)

ament_package()
