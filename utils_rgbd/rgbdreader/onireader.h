#ifndef ONIREADER_H
#define ONIREADER_H
#include "rgbdreader.h"
#include <openni2/OpenNI.h>
#include <memory>
//The depth needs to be registered to the color
class OniReader: public RGBDReader
{
    cv::Point3f move_depth_to_color_coords(cv::Point3f);
    cv::Mat undistort_depth(cv::Mat);
    cv::Mat undistort_color(cv::Mat);

    int num_frames=0;

    openni::Device dev;
    openni::VideoStream depth_vs,color_vs;
    openni::VideoMode color_vm;
    openni::PlaybackControl* pbc=NULL;
    
    openni::VideoFrameRef next_depth_frame,next_color_frame,curr_depth_frame,curr_color_frame;
    bool depth_frame_drop=false,color_frame_drop=false;
    bool from_file;
    enum StreamType{rgb,depth};
    StreamType first_stream;

public:
    bool getNextFrame(cv::Mat &depth, cv::Mat &color) override;
    bool readCurrFrame() override;
    bool readFrameAt(unsigned long long frame_num) override;
    bool getCurrFrame(cv::Mat &depth, cv::Mat &color);
    bool fetchNextFrame();
    openni::VideoStream &getColorStream();
    openni::VideoStream &getDepthStream();

    OniReader(std::string file_path);
    ~OniReader();
};

#endif // ONIREADER_H
