if(NOT MVCAM_SDK_PATH)
    set(MVCAM_SDK_PATH /opt/MVS CACHE PATH "MVS root path")
endif()

# find all include directories
find_path(MVS_INCLUDE_DIRS MvCameraControl.h HINTS ${MVCAM_SDK_PATH} PATH_SUFFIXES include)

# find all libraries
find_library(MVS_LIBRARY_MVCAMERACONTROL MvCameraControl HINTS ${MVCAM_SDK_PATH} PATH_SUFFIXES lib/64 lib/aarch64 lib/32)

set(MVS_LIBS ${MVS_LIBRARY_MVCAMERACONTROL})
# find all source files
#aux_source_directory()
#set(MVS_SOURCE )

find_package_handle_standard_args(MVS DEFAULT_MSG MVS_INCLUDE_DIRS MVS_LIBS #[[MVS_SOURCE]])