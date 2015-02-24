#include <DepthSense.hxx>

class DepthSenseDriver
{
    DepthSenseDriver();
    ~DepthSenseDriver();

private:
    DepthSense::Context _context; ///< DepthSense context

    DepthSense::Device _device;  ///< DepthSense device
    DepthSense::ColorNode _color; ///< DepthSense color node
    DepthSense::DepthNode _depth; ///< DepthSense depth node
};

