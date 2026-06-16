cmake_minimum_required(VERSION 3.8)
project(mutil_odom_shared)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(mutil_odom_shared_node
	src/odom_shared.cpp
)

target_include_directories(mutil_odom_shared_node PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

target_compile_features(mutil_odom_shared_node PUBLIC cxx_std_17)

ament_target_dependencies(mutil_odom_shared_node
	geometry_msgs
	nav_msgs
	rclcpp
	tf2
	tf2_ros
)

install(
	TARGETS mutil_odom_shared_node
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
