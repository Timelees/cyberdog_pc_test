cmake_minimum_required(VERSION 3.8)
project(emergency_stop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(protocol REQUIRED)

add_executable(emergency_stop_node src/emergency_stop.cpp)
ament_target_dependencies(emergency_stop_node rclcpp protocol)

install(TARGETS emergency_stop_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
