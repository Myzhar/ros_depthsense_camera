#include <DepthSense.hxx>

#include <ros/ros.h>
#include <csignal>
#include <list>
#include <image_transport/image_transport.h>

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

    DepthSenseDriver();
    ~DepthSenseDriver();

    bool init();
    void release();

    void run();

private:
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

    // Image transportation
    image_transport::ImageTransport _imgTr;
    image_transport::CameraPublisher _rgb_pub;

    DepthSense::Context _context; ///< DepthSense context

    std::list<DepthSense::Device> _devices;
    std::list<NodeInfo*> _nodeInfos;

    bool _initialized;
    bool _streaming;
    bool _error;

    static bool _stopping;

    bool _publish_tf;
};

