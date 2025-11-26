cmake_minimum_required(VERSION 3.10)
project(MyProject VERSION 1.0 LANGUAGES CXX)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(OpenCV REQUIRED HINTS ~/MyLibSrc/OpenCV/OpenCVLib/lib/cmake/opencv4)
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
add_executable(test 
    test.cpp
    ../../rm-main/src/HikCamera.cpp
    ../../rm-main/src/logger.cpp
    ../../rm-main/src/Detecor.cpp
    ../../rm-main/src/NumClassifier.cpp
    ../../rm-main/src/Solver.cpp
)
target_include_directories(test PUBLIC ${MVS_INCLUDE_DIRS})
target_link_libraries(test ${OpenCV_LIBS} fmt::fmt spdlog::spdlog ${LIBUSB_1.0_LIBRARIES} ${MVS_LIBS} openvino::runtime)


add_executable(negative 
    negative.cpp
    ../../rm-main/src/HikCamera.cpp
    ../../rm-main/src/logger.cpp
    ../../rm-main/src/Detecor.cpp
    ../../rm-main/src/NumClassifier.cpp
    ../../rm-main/src/Solver.cpp
)
target_include_directories(negative PUBLIC ${MVS_INCLUDE_DIRS})
target_link_libraries(negative ${OpenCV_LIBS} fmt::fmt spdlog::spdlog ${LIBUSB_1.0_LIBRARIES} ${MVS_LIBS} openvino::runtime)

add_executable(cvtorgb 
    cvtorgb.cpp
    ../../rm-main/src/HikCamera.cpp
    ../../rm-main/src/logger.cpp
    ../../rm-main/src/Detecor.cpp
    ../../rm-main/src/NumClassifier.cpp
    ../../rm-main/src/Solver.cpp
)
target_include_directories(cvtorgb PUBLIC ${MVS_INCLUDE_DIRS})
target_link_libraries(cvtorgb ${OpenCV_LIBS} fmt::fmt spdlog::spdlog ${LIBUSB_1.0_LIBRARIES} ${MVS_LIBS} openvino::runtime)






