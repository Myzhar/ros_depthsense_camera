cmake_minimum_required(VERSION 2.8.7)

project(depthsense_camera_ros)


###############################################################################
## Note by Myzhar                                                            ##
## ===============                                                           ##
##                                                                           ##
## If you get an exception error of the type:                                ##
## "ERROR: no enumerator found, some dll files are missing"                  ##
##                                                                           ##
## you must run this command in a terminal:                                  ##
##                                                                           ##
## $sudo ln -s /lib/x86_64-linux-gnu/libudev.so.1.3.5 /usr/lib/libudev.so.0  ##
##                                                                           ##
## To be able to access the video streams you need to add your user to the   ##
## "video" usergroup entering the following commands:                        ##
##                                                                           ##
## $sudo adduser ubuntu video [replace "ubuntu" with your current username]  ##
## $sudo chmod g+rw /dev/video*                                              ##
###############################################################################

set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    geometry_msgs
)

#########################################################
# INCLUDES

set(depthsense_INC_DIRS
    src/
    src/driver/include/
    /opt/softkinetic/DepthSenseSDK/include
)

# Specify locations of header files.
include_directories(
	${catkin_INCLUDE_DIRS} 
	${depthsense_INC_DIRS}
)
#########################################################

#########################################################
# SOURCES
set(depthsense_SRC
    src/depthsense_camera_node.cpp
    src/driver/src/depthsense_camera_driver.cpp
)
#########################################################

#########################################################
# EXECUTABLE
add_executable(
  depthsense_camera_node
  ${depthsense_SRC}
)

FIND_LIBRARY(DEPTHSENSE_LIBRARY DepthSense DepthSensePlugins turbojpeg /opt/softkinetic/DepthSenseSDK/lib/)

target_link_libraries(
  depthsense_camera_node
  ${catkin_LIBRARIES} ${DEPTHSENSE_LIBRARY}
)
#########################################################

