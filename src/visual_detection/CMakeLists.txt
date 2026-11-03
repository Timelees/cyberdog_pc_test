cmake_minimum_required(VERSION 3.16)

# 工程主体使用 C++17，包含检测库、服务端程序和客户端程序。
project(visual_detection LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# OpenCV 负责视频读取、图像预处理和绘制；Threads 是 muduo 及后台处理线程需要的依赖。
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# 同时支持 ONNXRUNTIME_ROOT 和 ONNXRUNTIME_DIR，方便不同机器使用不同环境变量名。
find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS
    ${ONNXRUNTIME_ROOT}/include
    ${ONNXRUNTIME_DIR}/include
    $ENV{ONNXRUNTIME_ROOT}/include
    $ENV{ONNXRUNTIME_DIR}/include
)

# 查找 ONNX Runtime 主库。CUDA provider 会由 ONNX Runtime 在运行时按需加载。
find_library(ONNXRUNTIME_LIBRARY
  NAMES onnxruntime
  HINTS
    ${ONNXRUNTIME_ROOT}/lib
    ${ONNXRUNTIME_DIR}/lib
    $ENV{ONNXRUNTIME_ROOT}/lib
    $ENV{ONNXRUNTIME_DIR}/lib
)

# muduo 拆成 net/base 两个库，服务端和客户端都依赖它们。
find_library(MUDUO_NET_LIBRARY NAMES muduo_net)
find_library(MUDUO_BASE_LIBRARY NAMES muduo_base)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR "ONNX Runtime not found. Set ONNXRUNTIME_ROOT or ONNXRUNTIME_DIR to the unpacked ONNX Runtime directory.")
endif()

if(NOT MUDUO_NET_LIBRARY OR NOT MUDUO_BASE_LIBRARY)
  message(FATAL_ERROR "muduo libraries not found. Install muduo or add its library path to CMAKE_PREFIX_PATH/LD_LIBRARY_PATH.")
endif()

# 公共库包含检测器、视频管线、JSON 序列化和 TCP 客户端/服务端逻辑。
add_library(dog_detector_lib
  src/DataConfig.cpp
  src/DetectionClient.cpp
  src/DetectionServer.cpp
  src/ResultJson.cpp
  src/SoccerTracker.cpp
  src/VideoDetectionPipeline.cpp
  src/YoloV8Detector.cpp
)

# 对外暴露 include 目录，同时把 ONNX Runtime 头文件路径传给所有依赖目标。
target_include_directories(dog_detector_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ONNXRUNTIME_INCLUDE_DIR}
)

# 链接 OpenCV、ONNX Runtime、muduo 和线程库。
target_link_libraries(dog_detector_lib
  PUBLIC
    ${OpenCV_LIBS}
    ${ONNXRUNTIME_LIBRARY}
    ${MUDUO_NET_LIBRARY}
    ${MUDUO_BASE_LIBRARY}
    Threads::Threads
)

# dog_detect 既支持命令行离线检测，也支持 --server 模式启动 TCP 服务端。
add_executable(dog_detect src/main.cpp)
target_link_libraries(dog_detect PRIVATE dog_detector_lib)

# dog_client 是 TCP 客户端，用于发送视频请求并实时接收每帧坐标。
add_executable(dog_client src/client_main.cpp)
target_link_libraries(dog_client PRIVATE dog_detector_lib)

add_executable(visual_detection_node src/ros_node.cpp)
target_link_libraries(visual_detection_node dog_detector_lib)
ament_target_dependencies(visual_detection_node ament_index_cpp cv_bridge geometry_msgs rclcpp sensor_msgs std_msgs)

add_executable(soccer_global_converter src/SoccerGlobalConverter.cpp)
target_include_directories(soccer_global_converter PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(soccer_global_converter ${OpenCV_LIBS})
ament_target_dependencies(soccer_global_converter apriltag_msgs geometry_msgs rclcpp sensor_msgs tf2)

install(TARGETS dog_detector_lib dog_detect dog_client visual_detection_node soccer_global_converter
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(FILES data.yaml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
ament_package()
