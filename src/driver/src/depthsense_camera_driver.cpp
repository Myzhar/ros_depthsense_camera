#include "depthsense_camera_driver.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

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
    _vertexRgb_pub = _nh.advertise<sensor_msgs::PointCloud2>("vertex_rgb_data", 1, false);

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
        _context.run(); // TODO add exception handling
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

    _depthIntrinsics = params.depthIntrinsics;
    _colorIntrinsics = params.colorIntrinsics;
    _extrinsics = params.extrinsics;

    ROS_INFO("depth intrinsics: width %d, height %d, fx %f, fy %f, cx %f, cy %f, k1 %f, k2 %f, k3 %f, p1 %f, p2 %f",
             _depthIntrinsics.width,
             _depthIntrinsics.height,
             _depthIntrinsics.fx,
             _depthIntrinsics.fy,
             _depthIntrinsics.cx,
             _depthIntrinsics.cy,
             _depthIntrinsics.k1,
             _depthIntrinsics.k2,
             _depthIntrinsics.k3,
             _depthIntrinsics.p1,
             _depthIntrinsics.p2);
    ROS_INFO("color intrinsics: width %d, height %d, fx %f, fy %f, cx %f, cy %f, k1 %f, k2 %f, k3 %f, p1 %f, p2 %f",
             _colorIntrinsics.width,
             _colorIntrinsics.height,
             _colorIntrinsics.fx,
             _colorIntrinsics.fy,
             _colorIntrinsics.cx,
             _colorIntrinsics.cy,
             _colorIntrinsics.k1,
             _colorIntrinsics.k2,
             _colorIntrinsics.k3,
             _colorIntrinsics.p1,
             _colorIntrinsics.p2);
    ROS_INFO("extrinsics: r11 %f, r12 %f, r13 %f, r21 %f, r22 %f, r23 %f, r31 %f, r32 %f, r33 %f, t1 %f, t2 %f, t3 %f",
             _extrinsics.r11,
             _extrinsics.r12,
             _extrinsics.r13,
             _extrinsics.r21,
             _extrinsics.r22,
             _extrinsics.r23,
             _extrinsics.r31,
             _extrinsics.r32,
             _extrinsics.r33,
             _extrinsics.t1,
             _extrinsics.t2,
             _extrinsics.t3);
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
    depthNode.setEnableUvMap( true ); // TODO handle data
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

    // Compressed images are ignored. Using ROS Image Transportation!

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

        _lastRgbMsgHeader.stamp = ros::Time::now();
        _lastRgbMsgHeader.seq = info->totalSampleCount;
        _lastRgbMsgHeader.frame_id = "rgb_frame";


        _lastRgbMsg.header = _lastRgbMsgHeader;

        _lastRgbMsg.width = info->width;
        _lastRgbMsg.height = info->height;
        _lastRgbMsg.encoding = sensor_msgs::image_encodings::BGR8;

        _lastRgbMsg.step = info->width * 3;

        int imgSize = _lastRgbMsg.height * _lastRgbMsg.step;
        _lastRgbMsg.data.resize( imgSize );

        memcpy( (uint8_t*)(&_lastRgbMsg.data[0]), (uint8_t*)(&rawimg[0]), imgSize );

        // >>>>> Camera info
        sensor_msgs::CameraInfo cam_info_msg;

        cam_info_msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

        cam_info_msg.D.resize(5);
        cam_info_msg.D[0] = _colorIntrinsics.k1;
        cam_info_msg.D[1] = _colorIntrinsics.k2;
        cam_info_msg.D[2] = _colorIntrinsics.k3;
        cam_info_msg.D[3] = _colorIntrinsics.p1;
        cam_info_msg.D[4] = _colorIntrinsics.p2;
        cam_info_msg.K.fill( 0.0 );
        cam_info_msg.K[0] = _colorIntrinsics.fx;
        cam_info_msg.K[2] = _colorIntrinsics.cx;
        cam_info_msg.K[4] = _colorIntrinsics.fy;
        cam_info_msg.K[5] = _colorIntrinsics.cy;
        cam_info_msg.K[8] = 1.0;

        cam_info_msg.R.fill( 0.0 );
        cam_info_msg.P.fill( 0.0 );
        cam_info_msg.P[0] = _colorIntrinsics.fx;
        cam_info_msg.P[2] = _colorIntrinsics.cx;
        cam_info_msg.P[5] = _colorIntrinsics.fy;
        cam_info_msg.P[6] = _colorIntrinsics.cy;
        cam_info_msg.P[10] = 1.0;

        cam_info_msg.header = _lastRgbMsgHeader;

        cam_info_msg.width = info->width;
        cam_info_msg.height = info->height;
        // <<<<< Camera info

        _rgb_pub.publish( _lastRgbMsg, cam_info_msg );

        if(_publish_tf)
        {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
            tf::Quaternion q;
            q.setRPY(-90.0*DEG2RAD, 0.0*DEG2RAD, -90.0*DEG2RAD);
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
    if( fp_vertex!=NULL && _vertex_pub.getNumSubscribers()>0 )
    {
        //prep a consistent header

        _lastPtCloudMsgHeader.stamp = ros::Time::now();
        _lastPtCloudMsgHeader.seq = info->totalSampleCount;
        _lastPtCloudMsgHeader.frame_id = "depth_frame";

        _lastPtCloud.header = _lastPtCloudMsgHeader;

        int ptsCount = info->width * info->height;

        _lastPtCloud.fields.resize(3);
        _lastPtCloud.fields[0].name = "z";
        _lastPtCloud.fields[0].offset = 0;
        _lastPtCloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        _lastPtCloud.fields[0].count = 1;
        _lastPtCloud.fields[1].name = "y";
        _lastPtCloud.fields[1].offset = sizeof(float);
        _lastPtCloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        _lastPtCloud.fields[1].count = 1;
        _lastPtCloud.fields[2].name = "x";
        _lastPtCloud.fields[2].offset = 2*sizeof(float);
        _lastPtCloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        _lastPtCloud.fields[2].count = 1;

        _lastPtCloud.width = info->width;
        _lastPtCloud.height = info->height;
        _lastPtCloud.is_dense = true;
        _lastPtCloud.is_bigendian = false;
        _lastPtCloud.point_step = 3 * sizeof(float);
        _lastPtCloud.row_step = _lastPtCloud.point_step * _lastPtCloud.width ;

        int cloudSize = ptsCount * _lastPtCloud.point_step;
        _lastPtCloud.data.resize( cloudSize );

        memcpy( (uint8_t*)(&_lastPtCloud.data[0]), (uint8_t*)(&fp_vertex[0]), cloudSize );

        _vertex_pub.publish( _lastPtCloud );

        // TODO send 2d virtual laser scan message
    }
    // <<<<< float Point Cloud (data in m)

    // >>>>> RGB Point Cloud (data in m)
    const DepthSense::UV* uv_map = (const DepthSense::UV*) data.uvMap;

    if( uv_map!=NULL && _vertexRgb_pub.getNumSubscribers()>0 )
    {
        //prep a consistent header

        _lastPtCloudMsgHeader.stamp = ros::Time::now();
        _lastPtCloudMsgHeader.seq = info->totalSampleCount;
        _lastPtCloudMsgHeader.frame_id = "depth_frame";

        _lastRGBPtCloud.header = _lastPtCloudMsgHeader;

        int ptsCount = info->width * info->height;

        _lastRGBPtCloud.fields.resize(4);
        _lastRGBPtCloud.fields[0].name = "z";
        _lastRGBPtCloud.fields[0].offset = 0;
        _lastRGBPtCloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        _lastRGBPtCloud.fields[0].count = 1;
        _lastRGBPtCloud.fields[1].name = "y";
        _lastRGBPtCloud.fields[1].offset = sizeof(float);
        _lastRGBPtCloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        _lastRGBPtCloud.fields[1].count = 1;
        _lastRGBPtCloud.fields[2].name = "x";
        _lastRGBPtCloud.fields[2].offset = 2*sizeof(float);
        _lastRGBPtCloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        _lastRGBPtCloud.fields[2].count = 1;
        _lastRGBPtCloud.fields[3].name = "rgb";
        _lastRGBPtCloud.fields[3].offset = 3*sizeof(float);
        _lastRGBPtCloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        _lastRGBPtCloud.fields[3].count = 1;

        _lastRGBPtCloud.width = info->width;
        _lastRGBPtCloud.height = info->height;
        _lastRGBPtCloud.is_dense = true;
        _lastRGBPtCloud.is_bigendian = false;
        _lastRGBPtCloud.point_step = 4*sizeof(float);
        _lastRGBPtCloud.row_step = _lastRGBPtCloud.point_step * _lastRGBPtCloud.width;

        int cloudSize = ptsCount * _lastRGBPtCloud.point_step;
        _lastRGBPtCloud.data.resize( cloudSize );

        float* ptCloudPtr = (float*)(&_lastRGBPtCloud.data[0]);

        uint8_t* rgbData = (uint8_t*)(&_lastRgbMsg.data[0]);

        for( int r=0; r<info->height; r++ )
        {
            for( int c=0; c<info->width; c++ )
            {
                int uvIdx = c + r*info->width;
                float f_u = uv_map[uvIdx].u;
                float f_v = uv_map[uvIdx].v;

                int idx = c*4+r*info->width*4;
                int vertIdx = c+r*info->width;

                if( f_u != -FLT_MAX && f_v != -FLT_MAX )
                {
                    ptCloudPtr[idx + 0] = fp_vertex[vertIdx].x; // Z
                    ptCloudPtr[idx + 1] = fp_vertex[vertIdx].y; // Y
                    ptCloudPtr[idx + 2] = fp_vertex[vertIdx].z; // X

                    RGBA px;
                    px.val = 0;

                    int u = (int)(f_u*(float)_lastRgbMsg.width);
                    int v = (int)(f_v*(float)_lastRgbMsg.height);

                    if( u<_lastRgbMsg.width && v<_lastRgbMsg.height )
                    {
                        int idxRgb = u*3+v*_lastRgbMsg.width*3;
                        px.color.b = rgbData[idxRgb + 0];
                        px.color.g = rgbData[idxRgb + 1];
                        px.color.r = rgbData[idxRgb + 2];
                        px.color.a = 255; // not really used
                    }

                    //ptCloudPtr[idx + 3] = *reinterpret_cast<float*>(&px.val); // RGBA
                    ptCloudPtr[idx + 3] = *reinterpret_cast<float*>(&px.val); // RGB
                }
                else
                {
                    ptCloudPtr[idx + 0] = -2.0f; // Z
                    ptCloudPtr[idx + 1] = -2.0f; // Y
                    ptCloudPtr[idx + 2] = -2.0f; // X
                    ptCloudPtr[idx + 3] = 0;
                }
            }
        }

        _vertexRgb_pub.publish( _lastRGBPtCloud );
    }
    // <<<<< RGB Point Cloud (data in m)

    if(_publish_tf)
    {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        tf::Quaternion q;
        q.setRPY(90.0*DEG2RAD, 0.0*DEG2RAD, 0.0*DEG2RAD);
        transform.setRotation(q);
        br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "world", "depth_frame") );
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
