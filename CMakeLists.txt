cmake_minimum_required(VERSION 3.5)
project(bgt60tr13c_ros2)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Complex64Array.msg"
  "msg/StampedFloat32MultiArray.msg"
  DEPENDENCIES std_msgs
)

ament_package()