cmake_minimum_required(VERSION 3.8)
project(arm_movement)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs)
find_package(geometry_msgs)
find_package(tf2)
find_package(brain_msgs)


add_executable(move_to_marker src/move_to_marker.cpp)
target_include_directories(move_to_marker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_to_marker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move_to_marker
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
)

add_executable(arm_brain src/arm_brain.cpp)
target_include_directories(arm_brain PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(arm_brain PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  arm_brain
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "tf2"
  "brain_msgs"
)

install(TARGETS 
  move_to_marker
  arm_brain
DESTINATION lib/${PROJECT_NAME})

# Install launch files.

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
