cmake_minimum_required(VERSION 3.8)
project(topic_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

option(TOPIC_VISUALIZATION_BUILD_ROS_TOPIC_VISUAL "Build ros_topic_visual_node (may ICE on GCC7)" ON)

set(_topic_visual_targets "")

if(TOPIC_VISUALIZATION_BUILD_ROS_TOPIC_VISUAL)
	add_executable(ros_topic_visual_node
		src/ros_topic_visual.cpp
	)

	target_include_directories(ros_topic_visual_node PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
	)

	target_compile_features(ros_topic_visual_node PUBLIC cxx_std_17)

	ament_target_dependencies(ros_topic_visual_node
		geometry_msgs
		nav_msgs
		rclcpp
		sensor_msgs
		tf2
		tf2_ros
		visualization_msgs
	)

	# Workaround: GCC 7 in cyberdog_img:1.0 can ICE on deeply nested lambdas in ros_topic_visual.cpp.
	# When this happens, set TOPIC_VISUALIZATION_BUILD_ROS_TOPIC_VISUAL=OFF and build tags_visual_node only.
	if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.0)
		target_compile_options(ros_topic_visual_node PRIVATE -O0 -fno-inline)
	endif()

	list(APPEND _topic_visual_targets ros_topic_visual_node)
endif()

add_executable(vins_visual_node
	src/vins_visual.cpp
)

target_include_directories(vins_visual_node PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

target_compile_features(vins_visual_node PUBLIC cxx_std_17)

ament_target_dependencies(vins_visual_node
	geometry_msgs
	lifecycle_msgs
	nav_msgs
	rclcpp
	sensor_msgs
	tf2
	tf2_geometry_msgs
	tf2_ros
)
list(APPEND _topic_visual_targets vins_visual_node)

add_executable(tags_visual_node
	src/tags_visual.cpp
)

target_include_directories(tags_visual_node PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

target_compile_features(tags_visual_node PUBLIC cxx_std_17)

ament_target_dependencies(tags_visual_node
	geometry_msgs
	nav_msgs
	rclcpp
	sensor_msgs
	tf2
	tf2_ros
	visualization_msgs
)
list(APPEND _topic_visual_targets tags_visual_node)

install(
	TARGETS ${_topic_visual_targets}
	DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY include/
	DESTINATION include
)

install(
	DIRECTORY config/
	DESTINATION share/${PROJECT_NAME}/config
)

install(
	DIRECTORY launch/
	DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()