cmake_minimum_required(VERSION 3.8)
project(robot_status_web)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  robot_status_web/status_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME status_server
)
install(DIRECTORY web config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md requirements.txt
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
