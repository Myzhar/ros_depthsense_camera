cmake_minimum_required(VERSION 2.8.7)

project(depthsense_camera_ros)

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

FIND_LIBRARY(DEPTHSENSE_LIBRARY DepthSense /opt/softkinetic/DepthSenseSDK/lib/)

target_link_libraries(
  depthsense_camera_node
  ${catkin_LIBRARIES} ${DEPTHSENSE_LIBRARY}
)
#########################################################

