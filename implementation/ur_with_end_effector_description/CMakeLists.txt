cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(ur_with_end_effector_description)

find_package(ament_cmake REQUIRED)

install(
	DIRECTORY launch rviz urdf config
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
