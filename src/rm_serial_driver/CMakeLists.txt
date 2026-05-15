cmake_minimum_required(VERSION 3.10)
project(rm_serial_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(libxr REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY src)

target_link_libraries(${PROJECT_NAME} libxr::xr)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright
       ament_cmake_uncrustify ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)

  file(GLOB TEST_SOURCES test/*.cpp)
  foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    ament_add_gtest(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME} ${PROJECT_NAME})
  endforeach()
endif()

rclcpp_components_register_node(
  ${PROJECT_NAME} PLUGIN rm_serial_driver::RMSerialDriver EXECUTABLE
  ${PROJECT_NAME}_node)

ament_auto_package(INSTALL_TO_SHARE launch config)
