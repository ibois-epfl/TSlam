#include "onireader.h"
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace std;

openni::VideoStream &OniReader::getColorStream(){
    return color_vs;
}

openni::VideoStream &OniReader::getDepthStream(){
    return depth_vs;
}

OniReader::OniReader(string file_path)
{
    depth_size=color_size=cv::Size(640,480);
    depth_dist_coeffs=rgb_dist_coeffs=cv::Matx<double,5,1>(0.262383, -0.953104, -0.005358, 0.002628, 1.163314);
    depth_cam_mat=rgb_cam_mat=cv::Matx33d(517.306408, 0,          318.643040,
                                          0,          516.469215, 255.313989,
                                          0,          0,          1);
    registered=true;

    openni::OpenNI::initialize();
    dev.open(file_path.c_str());

    if(!dev.isValid())
        throw runtime_error("The device is not valid or the oni file does not exist.");
    from_file=dev.isFile();
    pbc=dev.getPlaybackControl();
    pbc->setSpeed(-1);
    pbc->setRepeatEnabled(true);
    if(!from_file)
        throw runtime_error("Live OpenNI device is still not supported. You can record an oni file and use its path as input.");
    depth_vs.create(dev,openni::SENSOR_DEPTH);
    color_vs.create(dev,openni::SENSOR_COLOR);

    double depth_scale=depth_vs.getVideoMode().getResolutionX()/double(depth_size.width);
    depth_cam_mat*=depth_scale;
    depth_cam_mat(2,2)=1;
    depth_size=cv::Size(depth_vs.getVideoMode().getResolutionX(),depth_vs.getVideoMode().getResolutionY());

    double color_scale=color_vs.getVideoMode().getResolutionX()/double(color_size.width);
    rgb_cam_mat*=color_scale;
    rgb_cam_mat(2,2)=1;
    color_size=cv::Size(color_vs.getVideoMode().getResolutionX(),color_vs.getVideoMode().getResolutionY());

    cv::Mat empty_mat;
    cv::initUndistortRectifyMap(rgb_cam_mat,rgb_dist_coeffs,cv::Mat(),rgb_cam_mat,color_size,CV_16SC2,rgb_undistort_map,empty_mat);
    cv::initUndistortRectifyMap(depth_cam_mat,depth_dist_coeffs,cv::Mat(),depth_cam_mat,depth_size,CV_16SC2,depth_undistort_map,empty_mat);

    num_frames=pbc->getNumberOfFrames(depth_vs);

    color_vm=color_vs.getVideoMode();
    if(color_vm.getPixelFormat() != openni::PIXEL_FORMAT_RGB888 && color_vm.getPixelFormat() != openni::PIXEL_FORMAT_YUV422)
        throw runtime_error("The color pixel format is not supported. It must be RGB888 or YUV422");
    color_size=cv::Size(color_vm.getResolutionX(),color_vm.getResolutionY());

    auto depth_vm=depth_vs.getVideoMode();
    depth_size=cv::Size(depth_vm.getResolutionX(),depth_vm.getResolutionY());
    
    depth_vs.start();
    color_vs.start();
}

OniReader::~OniReader(){
    openni::OpenNI::shutdown();
}

cv::Mat OniReader::undistort_depth(cv::Mat depth){
    cv::Mat undistorted_depth;
    cv::remap(depth,undistorted_depth,depth_undistort_map,cv::Mat(),cv::INTER_NEAREST);
    return undistorted_depth;
}

cv::Mat OniReader::undistort_color(cv::Mat color){
    cv::Mat undistorted_color;
    cv::remap(color,undistorted_color,rgb_undistort_map,cv::Mat(),cv::INTER_NEAREST);
    return undistorted_color;
}

bool OniReader::readCurrFrame(){
    if(!curr_color_frame.isValid() || !curr_depth_frame.isValid())
        return false;

    if(color_vm.getPixelFormat()==openni::PIXEL_FORMAT_YUV422){
        curr_color=cv::Mat(curr_color_frame.getHeight(),curr_color_frame.getWidth(),CV_8UC2,(unsigned char*)curr_color_frame.getData(),curr_color_frame.getStrideInBytes());
        cv::cvtColor(curr_color,curr_color,CV_YUV2BGR_Y422);
    }
    else if(color_vm.getPixelFormat()==openni::PIXEL_FORMAT_RGB888){
        curr_color=cv::Mat(curr_color_frame.getHeight(),curr_color_frame.getWidth(),CV_8UC3,(unsigned char*)curr_color_frame.getData(),curr_color_frame.getStrideInBytes());
        cv::cvtColor(curr_color,curr_color,CV_RGB2BGR);
    }

    curr_depth=cv::Mat(curr_depth_frame.getHeight(),curr_depth_frame.getWidth(),CV_16UC1,(unsigned char*)curr_depth_frame.getData(),curr_depth_frame.getStrideInBytes());
    return true;
}

bool OniReader::readFrameAt(unsigned long long frame_num){

    const long long int offset=(long long int)frame_num-curr_frame_num;

    if(offset<0){
        while(fetchNextFrame());
        cout<<curr_color_frame.getFrameIndex()<<"-----------"<<endl;
        cout<<curr_depth_frame.getFrameIndex()<<"-----------"<<endl;

        depth_frame_drop=false;
        color_frame_drop=false;

        curr_frame_num=0;
        for(int i=0;i<frame_num;i++)
            if(!fetchNextFrame())
                return false;

    }
    else{
        for(int i=0;i<offset;i++)
            if(!fetchNextFrame())
                return false;
    }
    return readCurrFrame();
}

bool OniReader::getCurrFrame(cv::Mat &depth, cv::Mat &color){
    if(!readCurrFrame())
        return false;
    depth=curr_depth;
    color=curr_color;
    return true;
}

bool OniReader::fetchNextFrame(){
    curr_frame_num++;

    if(!depth_frame_drop)
        depth_vs.readFrame(&next_depth_frame);

    if(!color_frame_drop)
        color_vs.readFrame(&next_color_frame);

    if(curr_frame_num>0){
        unsigned int next_diff = std::abs(int64(next_color_frame.getTimestamp())-int64(next_depth_frame.getTimestamp()));
        unsigned int next_color_diff = std::abs(int64(next_color_frame.getTimestamp())-int64(curr_depth_frame.getTimestamp()));
        unsigned int next_depth_diff = std::abs(int64(next_depth_frame.getTimestamp())-int64(curr_color_frame.getTimestamp()));

        depth_frame_drop = next_diff > next_color_diff || (from_file && ( next_color_frame.getFrameIndex() < curr_color_frame.getFrameIndex() ) );
        color_frame_drop = next_diff > next_depth_diff || (from_file && ( next_depth_frame.getFrameIndex() < curr_depth_frame.getFrameIndex() ) );
    }

    if(depth_frame_drop && color_frame_drop)
        return false;

    if(!depth_frame_drop)
        curr_depth_frame=next_depth_frame;

    if(!color_frame_drop)
        curr_color_frame=next_color_frame;

    return true;
}

bool OniReader::getNextFrame(cv::Mat &depth, cv::Mat &color){
    if(!fetchNextFrame())
        return false;
    return getCurrFrame(depth,color);
}

cv::Point3f OniReader::move_depth_to_color_coords(cv::Point3f depth){
    return depth;
}
