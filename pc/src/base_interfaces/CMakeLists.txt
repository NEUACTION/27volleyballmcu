cmake_minimum_required(VERSION 3.8)
project(base_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Flag.msg"
  "msg/HeartBag.msg"
  "msg/Motor.msg"
  "msg/McuTransmit.msg"
  "msg/McuReceive.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
