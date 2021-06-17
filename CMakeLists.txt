cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 20)

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

add_executable(vicon_object_pose
	src/vicon2pose.cpp
	)
ament_target_dependencies(vicon_object_pose rclcpp geometry_msgs Eigen3)

add_executable(vicon2tf
	src/vicon2tf.cpp
	)
ament_target_dependencies(vicon2tf rclcpp tf2_msgs Eigen3)

install(TARGETS
	vicon2pose
	vicon2tf
	vicon_object_pose
	DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/launch
	)

ament_package()
