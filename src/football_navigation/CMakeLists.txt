cmake_minimum_required(VERSION 3.5)
project(football_navigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(protocol REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

set(dependencies
  action_msgs
  behaviortree_cpp_v3
  builtin_interfaces
  dwb_core
  dwb_msgs
  geometry_msgs
  nav2_behavior_tree
  nav2_costmap_2d
  nav2_core
  nav2_msgs
  nav2_util
  nav_msgs
  pluginlib
  protocol
  rcl_interfaces
  std_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  visualization_msgs
)

set(minimal_rclcpp_dependencies
  rclcpp
  geometry_msgs
)

set(trajectory_dependencies
  rclcpp
  geometry_msgs
  std_msgs
  tf2
)

set(team_role_assigner_dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
)

set(tracking_action_dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_msgs
  std_msgs
)

set(visualization_dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  protocol
  tf2
  tf2_geometry_msgs
  tf2_ros
)

set(geometry_dependencies
  geometry_msgs
)

add_library(football_geometry_lib SHARED
  src/core/football_geometry.cpp
)

set_target_properties(football_geometry_lib PROPERTIES
  CXX_VISIBILITY_PRESET default
  VISIBILITY_INLINES_HIDDEN OFF
)


ament_target_dependencies(football_geometry_lib
  ${geometry_dependencies}
)

target_link_libraries(football_geometry_lib
  tf2::tf2
)

add_library(football_navigation_core SHARED
  src/plugins/dynamic_obstacle_marker.cpp
)

ament_target_dependencies(football_navigation_core
  nav2_costmap_2d
  ${geometry_dependencies}
)

target_link_libraries(football_navigation_core
  football_geometry_lib
)

add_executable(football_goal_adapter
  src/control/football_goal_adapter_node.cpp
  src/control/football_goal_adapter.cpp
)

ament_target_dependencies(football_goal_adapter
  rclcpp
  nav2_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  ${geometry_dependencies}
)

target_link_libraries(football_goal_adapter
  football_geometry_lib
)

add_library(football_multi_robot_obstacle_layer SHARED
  src/plugins/multi_robot_obstacle_layer.cpp
)

ament_target_dependencies(football_multi_robot_obstacle_layer
  ${dependencies}
)

target_link_libraries(football_multi_robot_obstacle_layer
  football_navigation_core
)

target_compile_definitions(
  football_multi_robot_obstacle_layer
  PRIVATE
    BT_PLUGIN_EXPORT
  PUBLIC
    PLUGINLIB__DISABLE_BOOST_FUNCTIONS
)

add_executable(football_fake_ball_publisher
  src/simulation/football_fake_ball_publisher.cpp
)

ament_target_dependencies(football_fake_ball_publisher
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

target_link_libraries(football_fake_ball_publisher
  football_geometry_lib
)

add_executable(football_simulation_input_publisher
  src/simulation/football_simulation_input_publisher.cpp
)

ament_target_dependencies(football_simulation_input_publisher
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

target_link_libraries(football_simulation_input_publisher
  football_geometry_lib
)

add_executable(football_simulation_navigator
  src/simulation/football_simulation_navigator.cpp
)

ament_target_dependencies(football_simulation_navigator
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_msgs
  std_msgs
)

target_link_libraries(football_simulation_navigator
  football_geometry_lib
)

add_executable(football_fake_other_robot_publisher
  src/simulation/football_fake_other_robot_publisher.cpp
)

add_dependencies(
  football_fake_other_robot_publisher
  football_geometry_lib
)

ament_target_dependencies(football_fake_other_robot_publisher
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_msgs
)

target_link_libraries(football_fake_other_robot_publisher
  football_geometry_lib
)

# GCC 7.5 in the ARM64 build environment can crash while compiling the
# two large Football translation units. Keep the package ABI and compiler
# unchanged, but disable memory-intensive optimisation/debug bookkeeping
# only for those source files.
if(
  CMAKE_CXX_COMPILER_ID STREQUAL "GNU"
  AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "8.0"
)
  set_source_files_properties(
    src/control/football_goal_adapter.cpp
    src/plugins/multi_robot_obstacle_layer.cpp
    PROPERTIES
      COMPILE_FLAGS
        "-O0 -g0 -fno-lto -fno-var-tracking -fno-var-tracking-assignments -fno-inline-functions -fno-ipa-cp -fno-ipa-cp-clone -fno-ipa-sra"
  )

  set_property(
    TARGET football_fake_other_robot_publisher
    APPEND_STRING
    PROPERTY LINK_FLAGS
      " -Wl,--no-keep-memory -Wl,--reduce-memory-overheads"
  )

  set_property(
    TARGET football_goal_adapter
    APPEND_STRING
    PROPERTY LINK_FLAGS
      " -Wl,--no-keep-memory -Wl,--reduce-memory-overheads"
  )

  set_property(
    TARGET football_multi_robot_obstacle_layer
    APPEND_STRING
    PROPERTY LINK_FLAGS
      " -Wl,--no-keep-memory -Wl,--reduce-memory-overheads"
  )
endif()

add_executable(football_trajectory_adapter
  src/control/football_trajectory_adapter.cpp
)

ament_target_dependencies(football_trajectory_adapter
  ${trajectory_dependencies}
)

target_link_libraries(football_trajectory_adapter
  football_geometry_lib
)

add_executable(football_tracking_action_client
  src/control/football_tracking_action_client.cpp
)

ament_target_dependencies(football_tracking_action_client
  ${tracking_action_dependencies}
)

target_link_libraries(football_tracking_action_client
  football_geometry_lib
)

add_executable(football_team_role_assigner
  src/coordination/football_team_role_assigner.cpp
)

ament_target_dependencies(football_team_role_assigner
  ${team_role_assigner_dependencies}
)

target_link_libraries(football_team_role_assigner
  football_geometry_lib
)

add_executable(football_ball_fusion
  src/coordination/football_ball_fusion.cpp
)

ament_target_dependencies(football_ball_fusion
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
)

target_link_libraries(football_ball_fusion
  football_geometry_lib
)

add_executable(football_visualization_node
  src/visualization/football_visualization_node.cpp
)

target_link_libraries(football_visualization_node
  football_geometry_lib
)


ament_target_dependencies(football_visualization_node
  ${visualization_dependencies}
)

pluginlib_export_plugin_description_file(
  nav2_costmap_2d
  football_navigation_plugins.xml
)

pluginlib_export_plugin_description_file(
  nav2_core
  football_navigation_plugins.xml
)

pluginlib_export_plugin_description_file(dwb_core football_navigation_dwb_plugins.xml)

set(football_identity_sources
  CMakeLists.txt
  include/football_navigation/football_geometry.hpp
  include/football_navigation/football_goal_adapter.hpp
  include/football_navigation/multi_robot_obstacle_layer.hpp
  src/core/football_geometry.cpp
  src/control/football_goal_adapter.cpp
  src/control/football_goal_adapter_node.cpp
  src/simulation/football_fake_ball_publisher.cpp
  src/simulation/football_simulation_input_publisher.cpp
  src/simulation/football_simulation_navigator.cpp
  src/simulation/football_fake_other_robot_publisher.cpp
  src/coordination/football_ball_fusion.cpp
  src/control/football_trajectory_adapter.cpp
  src/control/football_tracking_action_client.cpp
  src/coordination/football_team_role_assigner.cpp
  src/visualization/football_visualization_node.cpp
  src/plugins/dynamic_obstacle_marker.cpp
  src/plugins/multi_robot_obstacle_layer.cpp
  launch/football_navigation.launch.py
  launch/football_robot_runtime.launch.py
  launch/football_single_robot_simulation.launch.py
  params/football_single_robot_simulation.yaml
  params/football_robot_runtime.yaml
  football_navigation_plugins.xml
  football_navigation_dwb_plugins.xml
)
set_property(DIRECTORY APPEND PROPERTY
  CMAKE_CONFIGURE_DEPENDS ${football_identity_sources}
)
set(football_identity_input "")
foreach(relative_path IN LISTS football_identity_sources)
  file(SHA256
    "${CMAKE_CURRENT_SOURCE_DIR}/${relative_path}"
    source_sha256
  )
  string(APPEND football_identity_input
    "${relative_path}:${source_sha256}\n"
  )
endforeach()
string(SHA256 football_source_digest "${football_identity_input}")
set(football_acceptance_script_sha256 "unavailable")
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/football_user_function_acceptance.py")
  file(SHA256
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/football_user_function_acceptance.py"
    football_acceptance_script_sha256
  )
endif()
# Navigation2 is provided by this workspace at src/thirdparty/navigation2.
# Do not make configuration depend on the former sibling navigation_bringup
# package: it is an optional runtime integration package and is not required
# to compile this plugin package.
get_filename_component(
  nav2_smac_planner_prefix
  "${nav2_smac_planner_DIR}/../../.."
  ABSOLUTE
)
set(
  nav2_smac_planner_2d_library
  "${nav2_smac_planner_prefix}/lib/libnav2_smac_planner_2d.so"
)
if(NOT EXISTS "${nav2_smac_planner_2d_library}")
  message(FATAL_ERROR
    "SmacPlanner2D runtime library not found: ${nav2_smac_planner_2d_library}"
  )
endif()
file(SHA256
  "${nav2_smac_planner_2d_library}"
  smac_planner_2d_sha256
)
execute_process(
  COMMAND git rev-parse HEAD
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  OUTPUT_VARIABLE football_source_commit
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
if(football_source_commit STREQUAL "")
  set(football_source_commit "unavailable")
endif()
string(TIMESTAMP football_configured_utc "%Y-%m-%dT%H:%M:%SZ" UTC)
file(WRITE
  "${CMAKE_CURRENT_BINARY_DIR}/football_build_identity.json"
  "{\n"
  "  \"commit\": \"${football_source_commit}\",\n"
  "  \"configured_utc\": \"${football_configured_utc}\",\n"
  "  \"source_digest\": \"${football_source_digest}\",\n"
  "  \"acceptance_script_sha256\": \"${football_acceptance_script_sha256}\",\n"
  "  \"smac_planner_2d_sha256\": \"${smac_planner_2d_sha256}\"\n"
  "}\n"
)

install(TARGETS
  football_geometry_lib
  football_navigation_core
  football_multi_robot_obstacle_layer
  football_goal_adapter
  football_fake_ball_publisher
  football_simulation_input_publisher
  football_simulation_navigator
  football_fake_other_robot_publisher
  football_trajectory_adapter
  football_tracking_action_client
  football_team_role_assigner
  football_ball_fusion
  football_visualization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  params
  rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(FILES
  football_navigation_plugins.xml
  football_navigation_dwb_plugins.xml
  "${CMAKE_CURRENT_BINARY_DIR}/football_build_identity.json"
  VERIFICATION.md
  DESTINATION share/${PROJECT_NAME}
)

# Some source distributions intentionally omit the deployment helper scripts.
# Do not make the C++ package unconfigurable when those optional files are absent.
file(GLOB football_helper_scripts CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.sh"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py"
)
if(football_helper_scripts)
  install(PROGRAMS
    ${football_helper_scripts}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_football_navigation
    test/test_football_navigation.cpp
  )

  ament_target_dependencies(test_football_navigation
    ${dependencies}
  )

  target_link_libraries(test_football_navigation
    football_navigation_core
  )

  ament_add_pytest_test(
    test_football_contract
    test/test_football_contract.py
  )

  ament_add_pytest_test(
    test_football_all_fake_contract
    test/test_football_all_fake_contract.py
  )
endif()

ament_export_include_directories(include)

ament_export_libraries(
  football_geometry_lib
  football_navigation_core
  football_multi_robot_obstacle_layer
)

ament_export_dependencies(
  ${dependencies}
)

ament_package()
