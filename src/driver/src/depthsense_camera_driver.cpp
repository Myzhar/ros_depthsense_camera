#include "depthsense_camera_driver.h"
#include <signal.h>

// >>>>> Quit MACRO
#define CONTEXT_QUIT(context)                           \
    do                                                      \
{                                                       \
    try {                                               \
    context.quit();                                 \
    }                                                   \
    catch (DepthSense::InvalidOperationException&)      \
{                                                   \
    }                                                   \
    } while (0)
// <<<<< Quit MACRO

sig_atomic_t volatile g_request_shutdown = 0;

void mySigintHandler(int sig)
{
    g_request_shutdown = 1;
}

DepthSenseDriver::DepthSenseDriver()
    : _initialized(false)
    , _streaming(false)
    , _error(false)
{

}


DepthSenseDriver::~DepthSenseDriver()
{
    release();
}

void DepthSenseDriver::init()
{
    if (_initialized)
    {
        return;
    }

    _context = DepthSense::Context::createStandalone();

    _context.deviceAddedEvent().connect(this, &DepthSenseDriver::onDeviceAdded);
    _context.deviceRemovedEvent().connect(this, &DepthSenseDriver::onDeviceRemoved);

    std::vector<DepthSense::Device> devices = _context.getDevices();

    for (unsigned int i = 0; i < devices.size(); i++)
    {
        onDeviceAdded(_context, devices[i]);

        std::vector<DepthSense::Node> nodes = devices[i].getNodes();
        for (unsigned int j = 0; j < nodes.size(); j++)
        {
            onNodeAdded(devices[i], nodes[j]);
        }
    }
    _initialized = true;
}

void DepthSenseDriver::release()
{

}

void DepthSenseDriver::run()
{
    init();

    if (!_error)
    {
        _context.run();
    }

    release();
    ros::shutdown();
}

void DepthSenseDriver::onDeviceRemoved(DepthSense::Context context, DepthSense::Device device)
{
    ROS_INFO_STREAM("Device removed: " << device.getModel() << " Serial: " << device.getSerialNumber() );
    _devices.remove(device);
}

void DepthSenseDriver::onDeviceAdded(DepthSense::Context context, DepthSense::Device device)
{
    if (_error)
        return;

    DepthSense::Device::Model model = device.getModel();
    DepthSense::Device::Capabilities caps = device.getCapabilities();

    ROS_INFO_STREAM("--- Device added ----------------------------------");
    ROS_INFO_STREAM( " Model:        " << DepthSense::Device::Model_toString(model) );
    ROS_INFO_STREAM( " Capabilities: " << DepthSense::Device::Capabilities_toString(caps) );
    ROS_INFO_STREAM( " Serial:       " << device.getSerialNumber() );

    DepthSense::StereoCameraParameters params;
    params = device.getStereoCameraParameters();

    DepthSense::IntrinsicParameters depthIntrinsics = params.depthIntrinsics;
    DepthSense::IntrinsicParameters colorIntrinsics = params.colorIntrinsics;
    DepthSense::ExtrinsicParameters extrinsics = params.extrinsics;

    ROS_INFO("depth intrinsics: width %d, height %d, fx %f, fy %f, cx %f, cy %f, k1 %f, k2 %f, k3 %f, p1 %f, p2 %f",
             depthIntrinsics.width,
             depthIntrinsics.height,
             depthIntrinsics.fx,
             depthIntrinsics.fy,
             depthIntrinsics.cx,
             depthIntrinsics.cy,
             depthIntrinsics.k1,
             depthIntrinsics.k2,
             depthIntrinsics.k3,
             depthIntrinsics.p1,
             depthIntrinsics.p2);
    ROS_INFO("color intrinsics: width %d, height %d, fx %f, fy %f, cx %f, cy %f, k1 %f, k2 %f, k3 %f, p1 %f, p2 %f",
             colorIntrinsics.width,
             colorIntrinsics.height,
             colorIntrinsics.fx,
             colorIntrinsics.fy,
             colorIntrinsics.cx,
             colorIntrinsics.cy,
             colorIntrinsics.k1,
             colorIntrinsics.k2,
             colorIntrinsics.k3,
             colorIntrinsics.p1,
             colorIntrinsics.p2);
    ROS_INFO("extrinsics: r11 %f, r12 %f, r13 %f, r21 %f, r22 %f, r23 %f, r31 %f, r32 %f, r33 %f, t1 %f, t2 %f, t3 %f",
             extrinsics.r11,
             extrinsics.r12,
             extrinsics.r13,
             extrinsics.r21,
             extrinsics.r22,
             extrinsics.r23,
             extrinsics.r31,
             extrinsics.r32,
             extrinsics.r33,
             extrinsics.t1,
             extrinsics.t2,
             extrinsics.t3);
    ROS_INFO("---------------------------------------------------");

    device.nodeAddedEvent().connect(this, &DepthSenseDriver::onNodeAdded);
    device.nodeRemovedEvent().connect(this, &DepthSenseDriver::onNodeRemoved);

    _devices.push_back(device);
}

void DepthSenseDriver::onNodeAdded(DepthSense::Device device, DepthSense::Node node)
{
    if (_error)
        return;

    //bool doRegister = false;

    int32_t width = 0;
    int32_t height = 0;
    float range = 0.0f;

    ROS_INFO_STREAM("--- Node added ------------------------------------");

    ROS_INFO( " Node type: %s"
              " VID:       %04x"
              " PID:       %04x"
              " Revision:  %04d"
              , node.getType().name().c_str()
              , node.getVID()
              , node.getPID()
              , node.getRevision() );

    if (node.is<DepthSense::DepthNode>())
    {
        DepthSense::DepthNode n = node.as<DepthSense::DepthNode>();

        DepthSense::DepthNode::Configuration configuration = n.getConfiguration();

        ROS_INFO_STREAM(" Available configurations:");
        std::vector<DepthSense::DepthNode::Configuration> configurations = n.getConfigurations();

        for (unsigned int i = 0; i < configurations.size(); i++)
        {
            ROS_INFO("    %s - %d fps - %s - saturation %s", DepthSense::FrameFormat_toString(configurations[i].frameFormat).c_str(),
                     configurations[i].framerate,
                     DepthSense::DepthNode::CameraMode_toString(configurations[i].mode).c_str(),
                     configurations[i].saturation ? "enabled" : "disabled");
        }

        n.setEnableDepthMap(false);
        n.setEnableVerticesFloatingPoint(true);
        n.setEnableUvMap(true);
        n.setEnableAccelerometer(true);
        bool doSetConfiguration = true;

        if (doSetConfiguration)
        {
            try
            {
                _context.requestControl(n, 0);
            }

            catch (DepthSense::Exception&)
            {
                ROS_INFO_STREAM("---------------------------------------------------");
                ROS_ERROR_STREAM("Error : Could not take control");

                _error = true;
                CONTEXT_QUIT(_context);

                return;
            }

            n.setConfidenceThreshold(100); // TODO PARAM ?

            try
            {
                n.setConfiguration(configuration);
                _context.releaseControl(n);
            }
            catch (std::exception&)
            {
                ROS_INFO_STREAM("---------------------------------------------------");
                ROS_INFO("Incorrect configuration :");
                ROS_INFO(" - Frame format: %s", DepthSense::FrameFormat_toString(configuration.frameFormat).c_str());
                ROS_INFO(" - Frame rate: %d fps", configuration.framerate);
                ROS_INFO(" - Mode: %s", DepthSense::DepthNode::CameraMode_toString(configuration.mode).c_str());
                ROS_INFO(" - Saturation: %s", configuration.saturation ? "enabled" : "disabled");
                _error = true;

                CONTEXT_QUIT(_context);
                return;
            }
        }

        ROS_INFO(" Depth node streaming enabled - current configuration:");
        ROS_INFO("    %s - %d fps - %s - saturation %s", DepthSense::FrameFormat_toString(configuration.frameFormat).c_str(),
                 configuration.framerate,
                 DepthSense::DepthNode::CameraMode_toString(configuration.mode).c_str(),
                 configuration.saturation ? "enabled" : "disabled");

        n.newSampleReceivedEvent().connect (this, &DepthSenseDriver::onNewDepthNodeSampleReceived);

        DepthSense::FrameFormat_toResolution(configuration.frameFormat, &width, &height);
        range = n.getRange();
    }
    else if ( node.is<DepthSense::ColorNode>() )
    {
        DepthSense::ColorNode c = node.as<DepthSense::ColorNode>();

        DepthSense::ColorNode::Configuration configuration = c.getConfiguration();

        ROS_INFO(" Available configurations:");
        std::vector<DepthSense::ColorNode::Configuration> configurations = c.getConfigurations();
        for (unsigned int i = 0; i < configurations.size(); i++)
        {
            ROS_INFO("    %s - %d fps - %s - %s", DepthSense::FrameFormat_toString(configurations[i].frameFormat).c_str(),
                     configurations[i].framerate,
                     DepthSense::CompressionType_toString(configurations[i].compression).c_str(),
                     DepthSense::PowerLineFrequency_toString(configurations[i].powerLineFrequency).c_str());
        }

        c.setEnableColorMap(true); // TODO PARAM
        c.setEnableCompressedData(false); // TODO PARAM
        configuration.frameFormat=DepthSense::FRAME_FORMAT_VGA;
        configuration.compression=DepthSense::COMPRESSION_TYPE_MJPEG;

        bool doSetConfiguration = true;

        if (doSetConfiguration)
        {
            try {
                _context.requestControl(c, 0);
            }

            catch (DepthSense::Exception&) {
                ROS_INFO("---------------------------------------------------");
                ROS_INFO("Error : Could not take control");
                _error = true;
                CONTEXT_QUIT(_context);
                return;
            }

            try {
                c.setConfiguration(configuration);
                _context.releaseControl(c);
            }
            catch (std::exception&) {
                ROS_INFO("---------------------------------------------------");
                ROS_INFO("Incorrect configuration :");
                ROS_INFO(" - Frame format: %s", DepthSense::FrameFormat_toString(configuration.frameFormat).c_str());
                ROS_INFO(" - Frame rate: %d fps", configuration.framerate);
                _error = true;
                CONTEXT_QUIT(_context);
                return;
            }
        }
        configuration = c.getConfiguration();
        ROS_INFO(" color node streaming enabled - current configuration:");
        ROS_INFO("    %s - %d fps - %s - %s", DepthSense::FrameFormat_toString(configuration.frameFormat).c_str(),
                 configuration.framerate,
                 DepthSense::CompressionType_toString(configuration.compression).c_str(),
                 DepthSense::PowerLineFrequency_toString(configuration.powerLineFrequency).c_str());

        c.newSampleReceivedEvent().connect(this, &DepthSenseDriver::onNewColorNodeSampleReceived);

        DepthSense::FrameFormat_toResolution(configuration.frameFormat, &width, &height);
    }
    ROS_INFO("---------------------------------------------------");

    if (node.is<DepthSense::UnsupportedNode>()) {
        return;
    }

    try {
        _context.registerNode(node);
    }

    catch(DepthSense::Exception& e) {
        ROS_ERROR("Could not register node : %s", e.what());
        exit(-1);
    }

    try {
        _context.startNodes();
        _streaming = true;
    }
    catch(DepthSense::Exception& e) {
        ROS_ERROR("Couldn't start streaming : %s", e.what());
        exit(-1);
    }
    if (!node.is<DepthSense::UnsupportedNode>())
    {
        NodeInfo* info = new NodeInfo;

        info->node = node;
        info->sampleCount = 0;
        info->totalSampleCount = 0;
        info->droppedSampleCount = 0;
        info->width = width;
        info->height = height;
        info->range = range;

        _nodeInfos.push_back(info);
    }

}

void DepthSenseDriver::onNodeRemoved(DepthSense::Device device, DepthSense::Node node)
{
    ROS_INFO_STREAM("Node removed");

    std::list<NodeInfo*>::iterator it;
    for (it = _nodeInfos.begin(); it != _nodeInfos.end(); it++)
    {
        if ((*it)->node == node)
        {
            NodeInfo* info = *it;
            _nodeInfos.remove(info);
            delete info;
            break;
        }
    }
}

void DepthSenseDriver::onNewColorNodeSampleReceived
(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data)

{
    const uint8_t* rawimg;
    const uint8_t* compressedimg;

    NodeInfo* info = findInfo(node);
    if (info == NULL) {
        return;
    }

    rawimg = (const uint8_t*) data.colorMap;
    //    compressedimg = (const uint8_t*) data.compressedData;

    if (info->totalSampleCount == 0) {
        info->sampleCount = 0;
    }

    ++info->totalSampleCount;
    ++info->sampleCount;

    //    std_msgs::Header msgheader;
    //    msgheader.stamp = ros::Time::now();
    //    msgheader.seq = info->totalSampleCount;
    //    msgheader.frame_id = "camera_color_frame";

    //    Mat rgbImg(info->height, info->width, CV_8UC3, (uint8_t*)rawimg);
    ////broadcast raw color image
    //    cv_bridge::CvImage rgb_msg;
    //    rgb_msg.header   = msgheader;
    //    rgb_msg.encoding = sensor_msgs::image_encodings::BGR8;
    //    rgb_msg.image    = rgbImg;
    //    raw_rgb.publish(rgb_msg.toImageMsg());

    ////convert rgb to hsv
    //    Mat hsvImg;
    //    cv::cvtColor(rgbImg, hsvImg, CV_BGR2HSV);

    ////split and process image in hsv color space
    //    vector<Mat> hsv_planes;
    //    split( hsvImg, hsv_planes );
    //    Mat h_plane;
    //    Mat h_plane2;
    //    Mat s_plane;
    //    threshold(hsv_planes[0], h_plane, 25, 255, THRESH_BINARY_INV);
    //    threshold(h_plane, h_plane2, 155, 255, THRESH_BINARY);
    //    threshold(hsv_planes[1], s_plane, 128, 255, THRESH_BINARY);

    ////write resultant mask to global object
    //    hsvmtx.lock();
    //    bitwise_and(h_plane2,s_plane,filteredhsv);
    //    hsvmtx.unlock();

    ////broadcast hsv filter image

    //    cv_bridge::CvImage hsvfilter_msg;
    //    hsvfilter_msg.header   = msgheader;
    //    hsvfilter_msg.encoding = sensor_msgs::image_encodings::MONO8;
    //    hsvfilter_msg.image    = filteredhsv;
    //    hsvfilter.publish(hsvfilter_msg.toImageMsg());


    ////reduce image to line and sum
    //    Mat hsvHistogram;
    //    Scalar hsvSummation;
    //    double maxval;
    //    Point maxidx;
    //    reduce(filteredhsv,hsvHistogram,0,CV_REDUCE_SUM,CV_32SC1);
    //    minMaxLoc(hsvHistogram, NULL, &maxval, NULL, &maxidx);
    //    hsvSummation = cv::sum(hsvHistogram);

    ////broadcast projected target
    //    geometry_msgs::PointStamped projected;
    //    projected.header = msgheader;
    //    projected.point.x = 1;
    //    projected.point.y = float(maxidx.x-320)/-587.f;
    //    projected.point.z = 0;
    //    if (hsvSummation[0] > 10000) {
    //        projected_target.publish(projected);
    //    }

    if (g_request_shutdown) {
        CONTEXT_QUIT(_context);
    }

}

void DepthSenseDriver::onNewDepthNodeSampleReceived
(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data)

{
    NodeInfo* info = findInfo(node);

    if (info == NULL) {
        return;
    }

    const DepthSense::FPVertex* floatVertex = (const DepthSense::FPVertex*) data.verticesFloatingPoint;
    const DepthSense::UV* uv = (const DepthSense::UV*) data.uvMap;
    const DepthSense::DepthNode::Acceleration acceleration = data.acceleration;


    if (info->totalSampleCount == 0) {
        info->sampleCount = 0;
    }

    ++info->totalSampleCount;
    ++info->sampleCount;

    ////prep a consistent header
    //    std_msgs::Header msgheader;
    //    msgheader.stamp = ros::Time::now();
    //    msgheader.seq = info->totalSampleCount;
    //    msgheader.frame_id = "camera_gimbal_frame";

    ////broadcast acclerometer data
    //    geometry_msgs::PointStamped accel;
    //    accel.header = msgheader;
    //    accel.point.x = data.acceleration.x;
    //    accel.point.y = data.acceleration.y;
    //    accel.point.z = data.acceleration.z;
    //    accel_data.publish(accel);

    ////make uvmap image from uv data
    //    Mat uvMap(info->height, info->width, CV_32FC2, (float *)uv );
    //    //broadcast uvmap image
    //    cv_bridge::CvImage uvmap_msg;
    //    uvmap_msg.header   = msgheader;
    //    uvmap_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
    //    uvmap_msg.image    = uvMap;
    //    raw_uvmap.publish(uvmap_msg.toImageMsg());

    ////pull in hsv data from camera thread
    //    hsvmtx.lock();
    //    Mat hsvmask = filteredhsv;
    //    hsvmtx.unlock();

    ////generate cloud of target color points
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudtargetmeasured(new pcl::PointCloud<pcl::PointXYZ>);
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
    //    std::vector<int> hsvindices;
    //    for (int i=0; i < uvMap.total(); ++i) {
    //        if (((float*)uv)[i*2] == -FLT_MAX) continue;
    //        if ( hsvmask.at<uchar>( floor(480*((float*)uv)[i*2+1])*640 + floor(640*((float*)uv)[i*2]) ) ){
    //            hsvindices.push_back(i);
    //            cloudtargetmeasured->push_back(pcl::PointXYZ(floatVertex[i].x,floatVertex[i].y,floatVertex[i].z));
    //        }
    //    }

    ////check if points fit spherical model
    //    std::vector<int> inliers;
    //    Eigen::VectorXf model_coefficients;
    //    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloudtargetmeasured));
    //    model_s->setRadiusLimits(0.08,0.13);
    //    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
    //    ransac.setDistanceThreshold (.01);
    //    ransac.computeModel();
    //    ransac.getInliers(inliers);
    //    ransac.getModelCoefficients(model_coefficients);
    //    pcl::copyPointCloud<pcl::PointXYZ>(*cloudtargetmeasured, inliers, *final);


    ////broadcast sphere points
    //    sensor_msgs::PointCloud2 targetpcl_out;
    //    pcl::toROSMsg(*final, targetpcl_out);
    //    targetpcl_out.header = msgheader;
    //    target_cloud.publish(targetpcl_out);

    ////broadcast sphere centroid if confidence is high
    ///*	if (final->points.size()) { ROS_INFO("%f %f %f %f -- %04d %04d -- %f %s", model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3],
    //        cloudtargetmeasured->size(),final->points.size(),
    //        (float)final->points.size()/(float)cloudtargetmeasured->points.size(),
    //        final->points.size() > 200 && (float)final->points.size()/(float)cloudtargetmeasured->points.size() > 0.75 ? "pass" : "fail"); }
    //*/
    //    geometry_msgs::PointStamped measured;
    //    if (final->points.size() > 200 && (float)final->points.size()/(float)cloudtargetmeasured->points.size() > 0.75) {
    //        measured.header = msgheader;
    //        measured.point.x = model_coefficients[0];
    //        measured.point.y = model_coefficients[1];
    //        measured.point.z = model_coefficients[2];
    //        measured_target.publish(measured);
    ////ROS_INFO("raw %d - radius %f - count %d - ratio %f\n", cloudtargetmeasured->points.size(), model_coefficients[3], final->points.size(),
    ////        (float)final->points.size()/(float)cloudtargetmeasured->points.size());
    //    }

    ////create obstacle map from pcl, assume the gimbal keeps the camera level
    //    Mat obstacle = Mat::zeros(41, 41, CV_8UC1);
    //    Mat obstaclethresh;
    //    int idx = 0;
    //    for (size_t i = 0; i < info->height*info->width; ++i) {
    //        if (floatVertex[i].z < 0.f || floatVertex[i].z > 2.f ||
    //            floatVertex[i].x < -1.f || floatVertex[i].x > 1.f ||
    //            floatVertex[i].y > 0.15 || floatVertex[i].y < -0.15) continue;

    //        int xidx = round(floatVertex[i].z*20);
    //        int yidx = round(-floatVertex[i].x*20)+20;

    //        if (obstacle.at<uchar>(xidx,yidx) < 255) obstacle.at<uchar>(xidx,yidx) += 1;
    //    }

    //    Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 8, 8 ) );
    //    threshold(obstacle, obstaclethresh, 10, 255, THRESH_BINARY);
    //    dilate(obstaclethresh,obstacle,element);
    //    cv_bridge::CvImage obstacle_msg;
    //    obstacle_msg.header   = msgheader;
    //    obstacle_msg.encoding = sensor_msgs::image_encodings::MONO8;
    //    obstacle_msg.image    = obstacle;
    //    obstacle_image.publish(obstacle_msg.toImageMsg());

    ////broadcast points
    ////totally destroys performance
    ///*
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
    //    for (size_t i = 0; i < info->height*info->width; ++i) {
    //        raw->push_back(pcl::PointXYZ(floatVertex[i].x,floatVertex[i].y,floatVertex[i].z));
    //    }

    //    sensor_msgs::PointCloud2 rawpcl_out;
    //    pcl::toROSMsg(*raw, rawpcl_out);
    //    rawpcl_out.header = msgheader;
    //    raw_cloud.publish(rawpcl_out);
    //*/

    if (g_request_shutdown) {
        CONTEXT_QUIT(_context);
    }

}

DepthSenseDriver::NodeInfo* DepthSenseDriver::findInfo(DepthSense::Node node)
{
    NodeInfo* info = NULL;
    std::list<NodeInfo*>::iterator it;
    for (it = _nodeInfos.begin(); it != _nodeInfos.end(); it++)
    {
        if ((*it)->node == node)
        {
            info = *it;
            break;
        }
    }
    return info;
}
