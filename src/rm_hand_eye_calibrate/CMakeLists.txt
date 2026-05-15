cmake_minimum_required(VERSION 3.8)
project(rm_hand_eye_calibrate)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(libxr REQUIRED)

ament_auto_find_build_dependencies()
ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY src)
ament_target_dependencies(${PROJECT_NAME} libxr)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>
                         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})

rclcpp_components_register_node(${PROJECT_NAME} PLUGIN HandEyeCalibrateNode
                                EXECUTABLE ${PROJECT_NAME}_node)

ament_auto_package(INSTALL_TO_SHARE launch config)
