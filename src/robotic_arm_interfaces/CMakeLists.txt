cmake_minimum_required(VERSION 3.8)
project(robotic_arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectInfo.msg"
  "srv/ListObjects.srv"
  "action/Pick.action"
  "action/Place.action"
  "action/MoveTo.action"
  "action/Home.action"
  "action/Reset.action"
  "action/OpenGripper.action"
  "action/CloseGripper.action"
)

ament_package()
