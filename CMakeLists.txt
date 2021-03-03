cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)

project(vicon_udp)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(vicon2pose
	src/vicon2pose.cpp
	)
ament_target_dependencies(vicon2pose rclcpp geometry_msgs Eigen3)

install(TARGETS
	vicon2pose
	DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/launch
	)

ament_package()
