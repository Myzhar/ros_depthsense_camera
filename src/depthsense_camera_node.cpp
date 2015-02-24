#include <stdlib.h>
#include <ros/ros.h>
#include <depthsense_camera_driver.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "depthsense_camera_node");

    ROS_INFO_STREAM("-----------------------------------\r");
    ROS_INFO_STREAM("    SoftKinetic DepthSense Node    \r");
    ROS_INFO_STREAM("-----------------------------------\r");

    DepthSenseDriver cameraDriver;

    cameraDriver.run();

    ros::spin();

    ROS_INFO_STREAM( "... shutting down complete." );

    return(EXIT_SUCCESS);
}
