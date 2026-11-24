cmake_minimum_required(VERSION 3.8)
project(mcu)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(base_interfaces REQUIRED)

add_executable(mcu src/main.cpp src/mcuNode.cpp src/serial.cpp)
# 与未来主控实现共享线格式定义，不复制两份长度/槽位常量。
target_include_directories(mcu PRIVATE include ${CMAKE_CURRENT_SOURCE_DIR}/../../../protocol)
target_compile_features(mcu PUBLIC cxx_std_17)
ament_target_dependencies(mcu rclcpp base_interfaces)
install(TARGETS mcu DESTINATION lib/${PROJECT_NAME})
ament_package()
