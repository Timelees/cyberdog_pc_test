cmake_minimum_required(VERSION 3.8)
project(keyboard_input)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(protocol REQUIRED)

add_executable(keyboard_input_node
	src/keyBoardInput.cpp
)

target_include_directories(keyboard_input_node PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

ament_target_dependencies(keyboard_input_node
	ament_index_cpp
	rclcpp
	protocol
)

install(
	TARGETS keyboard_input_node
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

ament_package()
