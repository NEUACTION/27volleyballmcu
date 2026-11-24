cmake_minimum_required(VERSION 3.8)
project(planner)

find_package(ament_cmake REQUIRED)
add_library(planner
  src/serve/plate_time.cpp
  src/serve/plate_distance.cpp
  src/catch/angle_five2.cpp
  src/catch/height_profile.cpp
  src/serve/arm_trajectory.cpp
  src/slide/position.cpp
  src/serve/arm_legacy.cpp
  src/serve/sequence.cpp
)
target_include_directories(planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(planner PUBLIC cxx_std_17)
ament_export_targets(plannerTargets HAS_LIBRARY_TARGET)
install(TARGETS planner EXPORT plannerTargets ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_package()
