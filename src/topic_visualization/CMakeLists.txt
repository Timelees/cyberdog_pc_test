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

install(
	TARGETS ros_topic_visual_node vins_visual_node tags_visual_node
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