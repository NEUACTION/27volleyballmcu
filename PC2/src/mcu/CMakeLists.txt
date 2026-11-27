cmake_minimum_required(VERSION 3.8)
project(mcu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(base_interfaces REQUIRED)
find_package(common_logger REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



include_directories(
  /usr/local/include   #添加这一行，对代码编译后会在该路径下生成所需的头文件
)
# 规定.so/.a库文件路径
link_directories(
  /usr/local/lib       #添加这一行，对代码编译后会在该路径下生成所需的库
)
set (dependencies
  rclcpp
  common_logger
)

# 需要将所有的cpp
FILE(GLOB SOURCE_LIST "src/*.cpp")

add_executable(mcu ${SOURCE_LIST})

target_include_directories(mcu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

  target_compile_features(mcu PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  ament_target_dependencies(
    mcu
    base_interfaces
    ${dependencies}
  )

install(TARGETS mcu
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
