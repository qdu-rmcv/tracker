cmake_minimum_required(VERSION 3.14)
project(libxr LANGUAGES C CXX ASM)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

set(LIBXR_SHARED_BUILD
    ON
    CACHE BOOL "" FORCE)
set(LIBXR_SYSTEM
    "linux"
    CACHE STRING "" FORCE)
set(LIBXR_DRIVER
    "linux"
    CACHE STRING "" FORCE)

add_subdirectory(libxr)

get_target_property(_xr_defs xr INTERFACE_COMPILE_DEFINITIONS)
if(_xr_defs)
  list(REMOVE_ITEM _xr_defs "EIGEN_NO_IO")
  set_target_properties(xr PROPERTIES INTERFACE_COMPILE_DEFINITIONS
                                      "${_xr_defs}")
endif()
target_compile_definitions(xr PRIVATE EIGEN_NO_IO)

set(LIBXR_SRC ${CMAKE_CURRENT_SOURCE_DIR}/libxr)

set(_libxr_system_dir "${LIBXR_SYSTEM}")
set(_libxr_driver_dir "${LIBXR_DRIVER}")
string(TOLOWER "${_libxr_system_dir}" _libxr_system_dir)
string(TOLOWER "${_libxr_driver_dir}" _libxr_driver_dir)

set_target_properties(xr PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "")

target_include_directories(xr PRIVATE ${LIBXR_SRC}/lib/Eigen)

target_link_libraries(xr PUBLIC Eigen3::Eigen)

set(_xr_public_build_include_dirs
    ${LIBXR_SRC}/src
    ${LIBXR_SRC}/src/core
    ${LIBXR_SRC}/src/system
    ${LIBXR_SRC}/src/utils
    ${LIBXR_SRC}/src/structure
    ${LIBXR_SRC}/src/middleware
    ${LIBXR_SRC}/src/middleware/message
    ${LIBXR_SRC}/src/driver
    ${LIBXR_SRC}/system/${_libxr_system_dir}
    ${LIBXR_SRC}/driver/${_libxr_driver_dir}
    ${LIBXR_SRC}/src/driver/usb
    ${LIBXR_SRC}/src/driver/usb/core
    ${LIBXR_SRC}/src/driver/usb/device
    ${LIBXR_SRC}/src/driver/usb/device/bos
    ${LIBXR_SRC}/src/driver/usb/device/cdc
    ${LIBXR_SRC}/src/driver/usb/device/dap
    ${LIBXR_SRC}/src/driver/usb/device/dfu
    ${LIBXR_SRC}/src/driver/usb/device/gsusb
    ${LIBXR_SRC}/src/driver/usb/device/hid
    ${LIBXR_SRC}/src/driver/usb/device/uac)

set(_xr_public_install_include_dirs
    include/libxr
    include/libxr/message
    include/libxr/usb
    include/libxr/usb/core
    include/libxr/usb/device
    include/libxr/usb/device/bos
    include/libxr/usb/device/cdc
    include/libxr/usb/device/dap
    include/libxr/usb/device/dfu
    include/libxr/usb/device/gsusb
    include/libxr/usb/device/hid
    include/libxr/usb/device/uac)

foreach(_dir IN LISTS _xr_public_build_include_dirs)
  if(EXISTS "${_dir}")
    target_include_directories(xr PUBLIC $<BUILD_INTERFACE:${_dir}>)
  endif()
endforeach()

foreach(_dir IN LISTS _xr_public_install_include_dirs)
  target_include_directories(xr PUBLIC $<INSTALL_INTERFACE:${_dir}>)
endforeach()

set(_header_dirs
    ${LIBXR_SRC}/src
    ${LIBXR_SRC}/src/core
    ${LIBXR_SRC}/src/system
    ${LIBXR_SRC}/src/utils
    ${LIBXR_SRC}/src/structure
    ${LIBXR_SRC}/src/middleware
    ${LIBXR_SRC}/src/driver
    ${LIBXR_SRC}/system/${_libxr_system_dir}
    ${LIBXR_SRC}/driver/${_libxr_driver_dir})
foreach(_dir ${_header_dirs})
  if(EXISTS "${_dir}")
    install(
      DIRECTORY "${_dir}/"
      DESTINATION include/libxr
      FILES_MATCHING
      PATTERN "*.hpp"
      PATTERN "*.h")
  endif()
endforeach()

install(
  DIRECTORY "${LIBXR_SRC}/src/driver/usb/"
  DESTINATION include/libxr
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN "*.h")

install(
  TARGETS xr
  EXPORT libxr_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(libxr_export HAS_LIBRARY_TARGET)
ament_export_include_directories(${_xr_public_install_include_dirs})
ament_export_libraries(xr)
ament_export_dependencies(Eigen3)

ament_package()
