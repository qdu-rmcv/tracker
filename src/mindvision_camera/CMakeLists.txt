cmake_minimum_required(VERSION 3.8)
project(mindvision_camera)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED src/mindvision_camera_node.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC include mindvisionSDK/include
                                                  ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# Select the bundled MindVision runtime library. You may override this manually,
# for example: colcon build --cmake-args -DMVSDK_ARCH=arm_softfp
if(NOT DEFINED MVSDK_ARCH)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
    set(MVSDK_ARCH x64)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(MVSDK_ARCH arm64)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i[3-6]86|x86")
    set(MVSDK_ARCH x86)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    set(MVSDK_ARCH arm)
  else()
    message(
      FATAL_ERROR
        "Unsupported host system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  endif()
endif()
set(MVSDK_ARCH
    ${MVSDK_ARCH}
    CACHE STRING "MindVision SDK library architecture directory")
set_property(CACHE MVSDK_ARCH PROPERTY STRINGS x64 x86 arm arm64 arm_softfp)

find_library(
  MVSDK_LIBRARY
  NAMES MVSDK libMVSDK.so
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/mindvisionSDK/lib/${MVSDK_ARCH}
        /usr/local/lib /usr/lib /lib /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
        /lib/${CMAKE_LIBRARY_ARCHITECTURE})

if(NOT MVSDK_LIBRARY)
  message(
    FATAL_ERROR
      "libMVSDK.so was not found. The uploaded SDK archive contains headers and demos, "
      "but no shared library. Either run the MindVision SDK install.sh so libMVSDK.so is "
      "available to the system linker, or copy libMVSDK.so into "
      "${CMAKE_CURRENT_SOURCE_DIR}/mindvisionSDK/lib/${MVSDK_ARCH}/ before building."
  )
endif()

message(STATUS "Using MindVision SDK library: ${MVSDK_LIBRARY}")
target_link_libraries(${PROJECT_NAME} ${MVSDK_LIBRARY})

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/mindvisionSDK/lib/${MVSDK_ARCH}/libMVSDK.so)
  install(DIRECTORY mindvisionSDK/lib/${MVSDK_ARCH}/ DESTINATION lib)
endif()

rclcpp_components_register_node(
  ${PROJECT_NAME} PLUGIN MindVisionCamera::MindVisionCameraNode EXECUTABLE
  ${PROJECT_NAME}_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
