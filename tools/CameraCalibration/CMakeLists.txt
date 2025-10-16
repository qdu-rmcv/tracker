cmake_minimum_required(VERSION 3.5) # Minimum CMake version required
# Set the project name and version
project(cv) # Project name is 'cv'


set(CMAKE_CXX_STANDARD 11)

set(CMAKE_EXPORT_COMPILE_COMMANDS True)

# Find all package that are required
find_package( OpenCV REQUIRED ) # Find OpenCV package
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(MVS REQUIRED)
find_package(PkgConfig REQUIRED)
# 找到 OpenVINO 包
# 需要先 source openvino/setupvars.sh
find_package(OpenVINO REQUIRED)

pkg_check_modules(LIBUSB_1.0 REQUIRED libusb-1.0)
# find_package(LibUSB 1.0 REQUIRED)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_executable(data 
    src/HikCamera.cpp
    src/logger.cpp
    src/data.cpp
)
add_executable(Calibration src/Calibration.cpp)

target_include_directories(data PUBLIC ${MVS_INCLUDE_DIRS})
target_link_libraries(data ${OpenCV_LIBS} fmt::fmt spdlog::spdlog ${LIBUSB_1.0_LIBRARIES} ${MVS_LIBS} )
# add the executable target

#link the OpenCV libraries to the executable
target_link_libraries(Calibration ${OpenCV_LIBS}) # Link OpenCV libraries to the cv executable








