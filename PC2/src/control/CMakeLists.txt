# ==========================================
# 区 1：定死的基础（建包时改一次，之后不动）
# ==========================================
cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
endif()

# ==========================================
# 区 2：查找依赖（常改！用了新包就加一行）
# ==========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rcutils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(base_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(common_logger REQUIRED)
find_package(Eigen3 REQUIRED)

# ==========================================
# 区 3：配置节点（常改！加节点或加依赖都在这里）
# ==========================================
# 1. 用 GLOB 自动抓取 src 下所有 cpp
FILE(GLOB SOURCE_LIST "src/*.cpp")

# 2. 生成一个叫 control 的可执行文件
add_executable(control ${SOURCE_LIST})

# 3. 指定头文件路径（为 Eigen 和 include 目录）
target_include_directories(control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${EIGEN3_INCLUDE_DIRS}
)

# 4. 指定 C++ 标准
target_compile_features(control PUBLIC c_std_99 cxx_std_17)

# 5. 把所有依赖绑给 control 这个可执行文件
# （你看，原来那个坑爹的 ${dependencies} 被拆成了明文，一目了然）
ament_target_dependencies(
  control
  rclcpp
  rcutils
  rclcpp_action
  geometry_msgs
  base_interfaces
  common_logger
  tf2_geometry_msgs
)

# 6. 安装可执行文件
install(TARGETS control
  DESTINATION lib/${PROJECT_NAME})

# 7. 安装 launch 文件夹
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ==========================================
# 区 4：收尾（死格式，永远放最后）
# ==========================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()