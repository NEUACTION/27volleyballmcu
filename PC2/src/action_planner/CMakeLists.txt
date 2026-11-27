cmake_minimum_required(VERSION 3.8)

#设置项目名称
project(action_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rcutils REQUIRED)
find_package(base_interfaces REQUIRED)
find_package(common_logger REQUIRED)

# === TF 相关依赖（新增）===
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# 如需 Eigen 下的变换，可再启用：
# find_package(tf2_eigen REQUIRED)

# 添加调试信息
message(STATUS "common_logger_INCLUDE_DIRS: ${common_logger_INCLUDE_DIRS}")
message(STATUS "common_logger_LIBRARIES: ${common_logger_LIBRARIES}")

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 添加依赖列表
set (dependencies 
  rclcpp
  rcutils
  rclcpp_action
  tf2               # 新增
  tf2_ros           # 新增：TransformBuffer/Listener/Broadcaster
  tf2_geometry_msgs # 新增：Pose/Vector/Stamped 等与 TF 的互转
  # tf2_eigen       # 若你在代码里用了 Eigen 与 TF 的互转，再放开
)

#匹配所有符合条件的文件并把路径存在SOURCE_LIST中
#FILE(GLOB 变量 模式)
#GLOB：FILE子命令，使用匹配模式
FILE(GLOB SOURCE_LIST "src/*.cpp")
##
#添加可执行文件
#add_executable(可执行文件名 源文件1 源文件2...)
add_executable(action_planner ${SOURCE_LIST})

target_include_directories(action_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(action_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#ROS2的CMAKE拓展指令，为目标指定依赖的ROS2包
#ament_target_dependencies(目标文件 依赖包1 依赖包2...)
ament_target_dependencies(
  action_planner
  base_interfaces
  common_logger
  ${dependencies}
)

install(TARGETS action_planner
  DESTINATION lib/${PROJECT_NAME})

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
