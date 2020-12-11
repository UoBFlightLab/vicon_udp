cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_STANDARD 17)

project(vicon_udp)

find_package(catkin REQUIRED
	COMPONENTS
	roscpp
	geometry_msgs
	)

find_package(Eigen3 REQUIRED NO_MODULE)

catkin_package()

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	)

add_executable(${PROJECT_NAME}_vicon2tf
	src/vicon2tf.cpp
	)
set_target_properties(${PROJECT_NAME}_vicon2tf PROPERTIES OUTPUT_NAME vicon2tf PREFIX "")
add_dependencies(${PROJECT_NAME}_vicon2tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_vicon2tf
  ${catkin_LIBRARIES}
  )

add_executable(${PROJECT_NAME}_vicon2pose
	src/vicon2pose.cpp
	)
set_target_properties(${PROJECT_NAME}_vicon2pose PROPERTIES OUTPUT_NAME vicon2pose PREFIX "")
add_dependencies(${PROJECT_NAME}_vicon2pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_vicon2pose
  ${catkin_LIBRARIES}
  )

install(
	DIRECTORY
		launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
	)
