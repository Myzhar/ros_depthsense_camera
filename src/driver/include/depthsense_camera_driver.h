#include <DepthSense.hxx>

#include <ros/ros.h>
#include <csignal>
#include <list>
#include <image_transport/image_transport.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

using namespace std;

class DepthSenseDriver
{
public:

    struct NodeInfo
    {
        DepthSense::Node node;
        int32_t sampleCount;
        int32_t totalSampleCount;
        int32_t droppedSampleCount;
        int32_t width;
        int32_t height;
        float range;
    };

    typedef union _RGBA
    {
        struct _color
        {
            uint8_t b;
            uint8_t g;
            uint8_t r;
            uint8_t a;
        } color;

        float val;
    } RGBA;

    DepthSenseDriver();
    ~DepthSenseDriver();

    bool init();
    void release();

    void run();



private:
    void loadParams();

    void onDeviceAdded(DepthSense::Context context, DepthSense::Device device);
    void onDeviceRemoved(DepthSense::Context context, DepthSense::Device device);

    void onNodeAdded(DepthSense::Device device, DepthSense::Node node);
    void onNodeRemoved(DepthSense::Device device, DepthSense::Node node);

    bool addDepthNode(DepthSense::Device device, DepthSense::Node node, int32_t& width, int32_t& height , float& range);
    bool addColorNode(DepthSense::Device device, DepthSense::Node node, int32_t& width, int32_t& height );
    bool addAudioNode(DepthSense::Device device, DepthSense::Node node);

    NodeInfo* findInfo(DepthSense::Node node);

    void onNewDepthNodeSampleReceived(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data);
    void onNewColorNodeSampleReceived(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data);

    void contextQuit();

    // >>>>> Ctrl+C handler
    /*! Ctrl+C handler
     */
    static void sighandler(int signo)
    {
        DepthSenseDriver::_stopping = (signo == SIGINT);
    }
    // <<<<< Ctrl+C handler

private:
    ros::NodeHandle _nh;
    ros::Publisher _vertex_pub;
    ros::Publisher _vertex_reg_pub;
    ros::Publisher _accel_pub;

    // >>>>> Image transportation RGB
    image_transport::ImageTransport _rgb_ImgTr;
    image_transport::CameraPublisher _rgb_pub;
    // <<<<< Image transportation RGB

    // >>>>> Image transportation Depth/Confidence
    image_transport::ImageTransport _depth_ImgTr;
    image_transport::CameraPublisher _depth_pub;
    image_transport::ImageTransport _confidence_ImgTr;
    image_transport::CameraPublisher _confidence_pub;
    // <<<<< Image transportation Depth/Confidence

    DepthSense::Context _context; ///< DepthSense context

    std::list<DepthSense::Device> _devices;
    std::list<NodeInfo*> _nodeInfos;

    bool _initialized;
    bool _streaming;
    bool _error;

    static bool _stopping;

    // >>>>> node params
    bool _publish_tf;           ///< Publish TF if true
    bool _enable_rgb;           ///< Publish RGB stream if true
    bool _enable_auto_wb;       ///< Enable RGB automatic white balance
    bool _enable_depth_confidence; ///< Publish Depth and Confidence stream if true
    int  _conf_thresh;          ///< Confidence threshold [DYN]
    int  _power_line_freq;      ///< Frequency of the light powerline [0-disabled,50hz,60hz]
    bool _enable_ptcloud;       ///< Publish 3d pointcloud if true.
    bool _enable_denoise;       ///< Enable denoise filter [DYN]
    bool _enable_registered;    ///< Publish 3d registered pointcloud if true and @ref _enable_rgb is true and @ref _enable_ptcloud is true
    bool _enable_accel;         ///< Publish accelerometer data if true

    // <<<<< node params

    // >>>>> Camera parameters
    DepthSense::IntrinsicParameters _depthIntrinsics;
    DepthSense::IntrinsicParameters _colorIntrinsics;
    DepthSense::ExtrinsicParameters _extrinsics;
    // <<<<< Camera parameters

    // >>>>> RGB ROS Message
    std_msgs::Header _lastRgbMsgHeader;
    sensor_msgs::Image _lastRgbMsg;
    // <<<<< RGB ROS Message

    // >>>>> Pointcloud ROS Message
    std_msgs::Header _lastPtCloudMsgHeader;
    sensor_msgs::PointCloud2 _lastPtCloud;
    sensor_msgs::PointCloud2 _lastRGBPtCloud;
    sensor_msgs::Imu _lastImuMsg;
    // <<<<< Pointcloud ROS Message
};

