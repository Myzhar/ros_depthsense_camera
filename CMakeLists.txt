cmake_minimum_required(VERSION 2.8.7)

project(ros_depthsense_camera)

###############################################################################
## Note by Myzhar                                                            ##
## ===============                                                           ##
##                                                                           ##
## If you get an exception error of the type:                                ##
## "ERROR: no enumerator found, some dll files are missing"                  ##
##                                                                           ##
## you must run this command in a terminal:                                  ##
##                                                                           ##
## 64 bit systems:                                                           ##
## $ sudo ln -s /lib/x86_64-linux-gnu/libudev.so.1.3.5 /usr/lib/libudev.so.0 ##
## 32 bit systems:                                                           ##
## $ sudo ln –s /lib/i386-linux-gnu/libudev.so.1 /usr/lib/libudev.so.0       ##
##                                                                           ##
## To be able to access the video streams you need to add your user to the   ##
## "video" usergroup entering the following commands:                        ##
##                                                                           ##
## $ sudo adduser ubuntu video [replace "ubuntu" with your current username] ##
## $ sudo chmod g+rw /dev/video*                                             ##
##                                                                           ##
## REMEMBER TO DELETE "CMakeCache.txt" if you want to change compilation     ##
## options                                                                   ##
###############################################################################

###############################################################################
## Options
option( DEBUG_ACTIVE "Enable Debug build" ON )

if(DEBUG_ACTIVE)
    MESSAGE( "Debug compilation active" )
    set(ROS_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Debug)
else()
    MESSAGE( "Release compilation active" )
    set(ROS_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Release)
endif()
###############################################################################

###############################################################################
## CUDA
find_package(CUDA)
if(CUDA_FOUND)
    try_run(RUN_RESULT_VAR COMPILE_RESULT_VAR
        ${CMAKE_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/cuda_test/has_cuda_gpu.c
        CMAKE_FLAGS
            -DINCLUDE_DIRECTORIES:STRING=${CUDA_TOOLKIT_INCLUDE}
            -DLINK_LIBRARIES:STRING=${CUDA_CUDART_LIBRARY}
        COMPILE_OUTPUT_VARIABLE COMPILE_OUTPUT_VAR
        RUN_OUTPUT_VARIABLE RUN_OUTPUT_VAR)
    message("${RUN_OUTPUT_VAR}") # Display number of GPUs found
    # COMPILE_RESULT_VAR is TRUE when compile succeeds
    # RUN_RESULT_VAR is zero when a GPU is found
    if(COMPILE_RESULT_VAR AND NOT RUN_RESULT_VAR)
        set(CUDA_HAVE_GPU TRUE CACHE BOOL "Whether CUDA-capable GPU is present")
    else()
        set(CUDA_HAVE_GPU FALSE CACHE BOOL "Whether CUDA-capable GPU is present")
    endif()
endif(CUDA_FOUND)
###############################################################################

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    geometry_msgs
    image_transport
)

###############################################################################
# INCLUDES
set(depthsense_INC_PATH
    src/driver/include/
)

set(depthsense_INC_DIRS
    src/
    ${depthsense_INC_PATH}
    /opt/softkinetic/DepthSenseSDK/include
)

# Specify locations of header files.
include_directories(
	${catkin_INCLUDE_DIRS} 
	${depthsense_INC_DIRS}
)
###############################################################################

###############################################################################
# CUDA SOURCES
if(CUDA_HAVE_GPU)
    add_definitions(-DWITH_CUDA)

    set(cuda_SRC_DIR src/driver/cuda)

    set(cuda_SRC ${cuda_SRC_DIR}/)

    SET(CUDA_NVCC_FLAGS "-arch=sm_13" CACHE STRING "nvcc flags" FORCE)

    #HW capability 1.3
    SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)

    SET(LIB_TYPE STATIC) #set the lib type
    CUDA_ADD_LIBRARY(Cuda2Dscan ${LIB_TYPE} ${cuda_SRC} )
endif(CUDA_HAVE_GPU)
###############################################################################

###############################################################################
# SOURCES
set(depthsense_SRC
    src/depthsense_camera_node.cpp
    src/driver/src/depthsense_camera_driver.cpp
)

set(depthsense_HDR
    ${depthsense_INC_PATH}/depthsense_camera_driver.h
)

###############################################################################

###############################################################################
# EXECUTABLE
add_executable(
  depthsense_camera_node
  ${depthsense_HDR}
  ${depthsense_SRC}
)

find_library(DEPTHSENSE_LIBRARY DepthSense DepthSensePlugins turbojpeg /opt/softkinetic/DepthSenseSDK/lib/)

if(CUDA_HAVE_GPU)
    add_dependencies(depthsense_camera_node Cuda2Dscan)
    target_link_libraries(depthsense_camera_node
        ${catkin_LIBRARIES} ${DEPTHSENSE_LIBRARY} Cuda2Dscan)
else()
    target_link_libraries(
        depthsense_camera_node
        ${catkin_LIBRARIES} ${DEPTHSENSE_LIBRARY}
    )
endif(CUDA_HAVE_GPU)
###############################################################################

