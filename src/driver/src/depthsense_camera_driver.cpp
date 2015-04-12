#include "depthsense_camera_driver.h"
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#ifndef DEG2RAD
#define DEG2RAD 0.017444444
#define RAD2DEG 57.324842225
#endif
#ifndef RAD2DEG
#define RAD2DEG 57.324842225
#endif

bool DepthSenseDriver::_stopping = false;

DepthSenseDriver::DepthSenseDriver()
    : _initialized(false)
    , _streaming(false)
    , _error(false)
    , _imgTr(_nh)
{
    struct sigaction sigAct;
    memset( &sigAct, 0, sizeof(sigAct) );
    sigAct.sa_handler = DepthSenseDriver::sighandler;
    sigaction(SIGINT, &sigAct, 0);

    _vertex_pub = _nh.advertise<sensor_msgs::PointCloud2>("vertex_data", 1, false);

    _rgb_pub = _imgTr.advertiseCamera("rgb_image", 1, false);


    _publish_tf = true;
}

DepthSenseDriver::~DepthSenseDriver()
{
    release();
}

void DepthSenseDriver::contextQuit()
{
    do
    {
        try
        {
            _context.quit();
        }
        catch (DepthSense::InvalidOperationException& e)
        {
            ROS_ERROR_STREAM( "Error quitting context: " << e.what() );
        }
    } while (0);
}

bool DepthSenseDriver::init()
{
    if( _initialized )
    {
        return true;
    }

    _context = DepthSense::Context::createStandalone();

    std::vector<DepthSense::Device> devices = _context.getDevices();
    int noDeviceCount = 0;

    do
    {
        devices = _context.getDevices();

        if( devices.size() == 0 )
        {
            if( noDeviceCount>60 || _stopping )
            {
                ROS_WARN_STREAM( "Aborting node... ");
                _error = true;
                return false;
            }

            noDeviceCount++;
            ROS_WARN_STREAM( "#" << noDeviceCount << " No connected device found... ");

            ros::Duration(1.0).sleep(); // sleep for 1 seconds
        }
    } while( devices.size()==0 );

    _context.deviceAddedEvent().connect(this, &DepthSenseDriver::onDeviceAdded);
    _context.deviceRemovedEvent().connect(this, &DepthSenseDriver::onDeviceRemoved);

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

    return true;
}

void DepthSenseDriver::release()
{
    if (!_initialized)
    {
        return;
    }

    ROS_INFO_STREAM( "Stopping device...");

    std::vector<DepthSense::Node> registeredNodes = _context.getRegisteredNodes();

    for (unsigned int i = 0; i < registeredNodes.size(); i++)
    {
        if (registeredNodes[i].is<DepthSense::DepthNode>())
        {
            DepthSense::DepthNode n = registeredNodes[i].as<DepthSense::DepthNode>();
            n.newSampleReceivedEvent().disconnect
                    (this, &DepthSenseDriver::onNewDepthNodeSampleReceived);
        }
    }

    std::list<DepthSense::Device>::iterator it;

    if (_streaming)
    {
        _context.stopNodes();
    }

    for (it = _devices.begin(); it != _devices.end(); it++)
    {
        DepthSense::Device device = *it;
        device.nodeRemovedEvent().disconnect(this, &DepthSenseDriver::onNodeRemoved);
        device.nodeAddedEvent().disconnect(this, &DepthSenseDriver::onNodeAdded);
    }

    _context.deviceRemovedEvent().disconnect(this, &DepthSenseDriver::onDeviceRemoved);
    _context.deviceAddedEvent().disconnect(this, &DepthSenseDriver::onDeviceAdded);

    //CONTEXT_QUIT(_context);
    contextQuit();

    while (!_nodeInfos.empty())
    {
        NodeInfo* info = _nodeInfos.front();
        _nodeInfos.pop_front();
        delete info;
    }

    _devices.clear();
    _initialized = false;
}

void DepthSenseDriver::run()
{
    init();

    if (!_error)
    {
        _context.run();
    }

    ROS_INFO_STREAM( "DepthSense context stopped... ");

    release();

    ROS_INFO_STREAM( "Stopping node... ");

    ros::shutdown();

    ROS_INFO_STREAM( "... done.");
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

bool DepthSenseDriver::addDepthNode(DepthSense::Device device, DepthSense::Node node, int32_t& width, int32_t& height , float& range)
{

    ROS_INFO_STREAM("--- New Depth Node  ------------------------------------");

    ROS_INFO( " Node type: %s - VID: %04x - PID: %04x - Revision: %04d"
              , node.getType().name().c_str()
              , node.getVID()
              , node.getPID()
              , node.getRevision() );

    DepthSense::DepthNode depthNode = node.as<DepthSense::DepthNode>();

    DepthSense::DepthNode::Configuration configuration = depthNode.getConfiguration();

    ROS_INFO_STREAM(" Available configurations:");
    std::vector<DepthSense::DepthNode::Configuration> configurations = depthNode.getConfigurations();

    for(unsigned int i = 0; i < configurations.size(); i++)
    {
        ROS_INFO("    %s - %d fps - %s - saturation %s",
                 DepthSense::FrameFormat_toString(configurations[i].frameFormat).c_str(),
                 configurations[i].framerate,
                 DepthSense::DepthNode::CameraMode_toString(configurations[i].mode).c_str(),
                 configurations[i].saturation ? "enabled" : "disabled");
    }

    // >>>>> Node data enabling
    depthNode.setEnableAccelerometer( false ); // TODO handle data
    depthNode.setEnableConfidenceMap( false ); // TODO handle data
    depthNode.setEnableDepthMap( false ); // TODO handle data
    depthNode.setEnableDepthMapFloatingPoint( false ); // TODO handle data
    depthNode.setEnablePhaseMap( false ); // TODO handle data
    depthNode.setEnableUvMap( false ); // TODO handle data
    depthNode.setEnableVertices( false ); // TODO handle data
    depthNode.setEnableVerticesFloatingPoint( true ); // TODO handle data
    // <<<<< Node data enabling

    bool doSetConfiguration = true;

    if (doSetConfiguration)
    {
        try
        {
            _context.requestControl(depthNode, 0);
        }

        catch (DepthSense::Exception&)
        {
            ROS_INFO_STREAM("---------------------------------------------------");
            ROS_ERROR_STREAM("Error : Could not take control");

            _error = true;
            //CONTEXT_QUIT(_context);
            contextQuit();

            return false;
        }

        // >>>>> Parameters
        depthNode.setConfidenceThreshold(60); // TODO create param
        depthNode.setEnableDenoising( true ); // TODO create param
        configuration.framerate = 60; // TODO create param
        // <<<<< Parameters

        try
        {
            depthNode.setConfiguration(configuration);
            _context.releaseControl(depthNode);
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

            //CONTEXT_QUIT(_context);
            contextQuit();
            return false;
        }
    }

    ROS_INFO(" Depth node streaming enabled - current configuration:");
    ROS_INFO("    %s - %d fps - %s - saturation %s", DepthSense::FrameFormat_toString(configuration.frameFormat).c_str(),
             configuration.framerate,
             DepthSense::DepthNode::CameraMode_toString(configuration.mode).c_str(),
             configuration.saturation ? "enabled" : "disabled");

    depthNode.newSampleReceivedEvent().connect (this, &DepthSenseDriver::onNewDepthNodeSampleReceived);

    DepthSense::FrameFormat_toResolution(configuration.frameFormat, &width, &height);
    range = depthNode.getRange();

    ROS_INFO("---------------------------------------------------");
    return true;
}

bool DepthSenseDriver::addColorNode(DepthSense::Device device, DepthSense::Node node, int32_t &width, int32_t &height )
{
    ROS_INFO_STREAM("--- New Color Node  ------------------------------------");

    ROS_INFO( " Node type: %s - VID: %04x - PID: %04x - Revision: %04d  \n"
              , node.getType().name().c_str()
              , node.getVID()
              , node.getPID()
              , node.getRevision() );

    DepthSense::ColorNode colorNode = node.as<DepthSense::ColorNode>();

    DepthSense::ColorNode::Configuration configuration = colorNode.getConfiguration();

    ROS_INFO(" Available configurations:");
    std::vector<DepthSense::ColorNode::Configuration> configurations = colorNode.getConfigurations();
    for (unsigned int i = 0; i < configurations.size(); i++)
    {
        ROS_INFO("    %s - %d fps - %s - %s", DepthSense::FrameFormat_toString(configurations[i].frameFormat).c_str(),
                 configurations[i].framerate,
                 DepthSense::CompressionType_toString(configurations[i].compression).c_str(),
                 DepthSense::PowerLineFrequency_toString(configurations[i].powerLineFrequency).c_str());
    }

    colorNode.setEnableColorMap(true); // TODO PARAM
    colorNode.setEnableCompressedData(false); // TODO PARAM

    configuration.frameFormat=DepthSense::FRAME_FORMAT_VGA;
    configuration.compression=DepthSense::COMPRESSION_TYPE_MJPEG;

    bool doSetConfiguration = true;

    if (doSetConfiguration)
    {
        try {
            _context.requestControl(colorNode, 0);
        }

        catch (DepthSense::Exception&) {
            ROS_INFO("---------------------------------------------------");
            //CONTEXT_QUIT(_context);
            contextQuit();
            return false;
        }
        
        //colorNode.setExposureAuto( DepthSense::EXPOSURE_AUTO_APERTURE_PRIORITY);
        //colorNode.setExposureAutoPriority( false );  // TODO create param
        colorNode.setWhiteBalanceAuto( true );  // TODO create param

        try {
            colorNode.setConfiguration(configuration);
            _context.releaseControl(colorNode);
        }
        catch (std::exception&) {
            ROS_INFO("---------------------------------------------------");
            ROS_INFO("Incorrect configuration :");
            ROS_INFO(" - Frame format: %s", DepthSense::FrameFormat_toString(configuration.frameFormat).c_str());
            ROS_INFO(" - Frame rate: %d fps", configuration.framerate);
            _error = true;
            //CONTEXT_QUIT(_context);
            contextQuit();
            return false;
        }
    }
    configuration = colorNode.getConfiguration();
    ROS_INFO(" color node streaming enabled - current configuration:");
    ROS_INFO("    %s - %d fps - %s - %s", DepthSense::FrameFormat_toString(configuration.frameFormat).c_str(),
             configuration.framerate,
             DepthSense::CompressionType_toString(configuration.compression).c_str(),
             DepthSense::PowerLineFrequency_toString(configuration.powerLineFrequency).c_str());

    colorNode.newSampleReceivedEvent().connect(this, &DepthSenseDriver::onNewColorNodeSampleReceived);

    DepthSense::FrameFormat_toResolution(configuration.frameFormat, &width, &height);

    ROS_INFO("---------------------------------------------------");

    return true;
}

bool DepthSenseDriver::addAudioNode(DepthSense::Device device, DepthSense::Node node )
{
    ROS_INFO_STREAM("--- New Audio Node  ------------------------------------");

    ROS_INFO( " Node type: %s - VID: %04x - PID: %04x - Revision: %04d"
              , node.getType().name().c_str()
              , node.getVID()
              , node.getPID()
              , node.getRevision() );

    ROS_INFO_STREAM("--- Audio node not yet supported -------");

    ROS_INFO("---------------------------------------------------");

    return false;
}

void DepthSenseDriver::onNodeAdded(DepthSense::Device device, DepthSense::Node node)
{
    if (_error)
        return;

    int32_t width = 0;
    int32_t height = 0;
    float range = 0.0f;

    if( node.is<DepthSense::AudioNode>() )
    {
        if( !addAudioNode( device, node ) )
            return;
    }

    if (node.is<DepthSense::DepthNode>())
    {
        if( !addDepthNode( device, node, width, height, range ) )
            return;
    }

    if ( node.is<DepthSense::ColorNode>() )
    {
        if( !addColorNode( device, node, width, height ) )
            return;
    }

    if (node.is<DepthSense::UnsupportedNode>())
    {
        return;
    }

    try
    {
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

void DepthSenseDriver::onNewColorNodeSampleReceived( DepthSense::ColorNode node,
                                                     DepthSense::ColorNode::NewSampleReceivedData data)
{
    if (_stopping)
    {
        ROS_INFO_STREAM( "Color sample received while node is closing..." );
        //CONTEXT_QUIT(_context);
        contextQuit();
        return;
    }

    NodeInfo* info = findInfo(node);
    if (info == NULL) {
        return;
    }



    // TODO Handle compressed images

    //const uint8_t* compressedimg;
    //compressedimg = (const uint8_t*) data.compressedData;

    if (info->totalSampleCount == 0) {
        info->sampleCount = 0;
    }

    ++info->totalSampleCount;
    ++info->sampleCount;
    
    // >>>>> RGB frame
    const uint8_t* rawimg = (const uint8_t*) data.colorMap;

    if( rawimg!=NULL )
    {

        //prep a consistent header
        std_msgs::Header msgheader;
        msgheader.stamp = ros::Time::now();
        msgheader.seq = info->totalSampleCount;
        msgheader.frame_id = "rgb_frame";

        sensor_msgs::Image msg;
        msg.header = msgheader;

        msg.width = info->width;
        msg.height = info->height;
        msg.encoding = sensor_msgs::image_encodings::BGR8; // TODO verify if BGR or RGB

        msg.step = info->width * 3;

        int imgSize = msg.height * msg.step;
        msg.data.resize( imgSize );

        memcpy( (uint8_t*)(&msg.data[0]), (uint8_t*)(&rawimg[0]), imgSize );

        // >>>>> Camera info
        sensor_msgs::CameraInfo cam_info_msg;

        cam_info_msg.D.resize(5);
        cam_info_msg.D[0] = 0.0;
        cam_info_msg.K.fill( 0.0 );
        cam_info_msg.R.fill( 0.0 );
        cam_info_msg.P.fill( 0.0 );
        cam_info_msg.P[0] = 1.0;
        cam_info_msg.P[5] = 1.0;
        cam_info_msg.P[10] = 1.0;

        cam_info_msg.header = msgheader;

        cam_info_msg.width = info->width;
        cam_info_msg.height = info->height;
        // <<<<< Camera info

        _rgb_pub.publish( msg, cam_info_msg );

        if(_publish_tf)
        {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
            tf::Quaternion q;
            q.setRPY(90.0*DEG2RAD, 0.0*DEG2RAD, 90.0*DEG2RAD);
            transform.setRotation(q);
            br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "world", "rgb_frame") );
        }
    }

    // <<<<< RGB frame

}

void DepthSenseDriver::onNewDepthNodeSampleReceived( DepthSense::DepthNode node,
                                                     DepthSense::DepthNode::NewSampleReceivedData data)
{
    if (_stopping)
    {
        ROS_INFO_STREAM( "Depth sample received while node is closing..." );
        //CONTEXT_QUIT(_context);
        contextQuit();
        return;
    }

    NodeInfo* info = findInfo(node);

    if (info == NULL) {
        return;
    }

    if (info->totalSampleCount == 0) {
        info->sampleCount = 0;
    }

    ++info->totalSampleCount;
    ++info->sampleCount;

    // ROS_INFO_STREAM( "Received depth frame #" << info->sampleCount );

    //const DepthSense::FPVertex* floatVertex = (const DepthSense::FPVertex*) data.verticesFloatingPoint;
    //const DepthSense::UV* uv = (const DepthSense::UV*) data.uvMap;
    //const DepthSense::DepthNode::Acceleration acceleration = data.acceleration;

    // >>>>> float Point Clouds (data in m)
    const DepthSense::FPVertex* fp_vertex = (const DepthSense::FPVertex*) data.verticesFloatingPoint;
    if( fp_vertex!=NULL )
    {
        //prep a consistent header
        std_msgs::Header msgheader;
        msgheader.stamp = ros::Time::now();
        msgheader.seq = info->totalSampleCount;
        msgheader.frame_id = "depth_frame";

        sensor_msgs::PointCloud2 msg;
        msg.header = msgheader;

        int ptsCount = info->width * info->height;

        msg.fields.resize(3);
        msg.fields[0].name = "x";
        msg.fields[0].offset = 0;
        msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[0].count = ptsCount;
        msg.fields[1].name = "y";
        msg.fields[1].offset = sizeof(float);
        msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[1].count = ptsCount;
        msg.fields[2].name = "z";
        msg.fields[2].offset = 2*sizeof(float);
        msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[2].count = ptsCount;

        msg.width = info->width;
        msg.height = info->height;
        msg.is_dense = true;
        msg.is_bigendian = false;
        msg.point_step = sizeof(float) * 3;
        msg.row_step = msg.point_step * msg.width;

        int cloudSize = ptsCount * msg.point_step;
        msg.data.resize( cloudSize );

        memcpy( (uint8_t*)(&msg.data[0]), (uint8_t*)(&fp_vertex[0]), cloudSize );

        _vertex_pub.publish( msg );

        if(_publish_tf)
        {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
            tf::Quaternion q;
            q.setRPY(90.0*DEG2RAD, 0.0*DEG2RAD, 90.0*DEG2RAD);
            transform.setRotation(q);
            br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "world", "depth_frame") );
        }

        // TODO send 2d virtual laser scan message
    }
    // <<<<< float Point Clouds (data in m)

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
