#include "tslamtypes.h"
#include "tslam.h"
#include "basictypes/debug.h"
#include "mapviewer.h"
#include "basictypes/timers.h"
#include "map.h"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "stdint.h"

void printHelp(cv::Mat &im)
{
    float fs=float(im.cols)/float(1200);

    cv::putText(im,"'s': start/stop recording",cv::Point(10,fs*80),cv::FONT_HERSHEY_SIMPLEX,fs*0.5f,cv::Scalar(125,255,255));
    cv::putText(im,"'p': pause recording",cv::Point(10,fs*100),cv::FONT_HERSHEY_SIMPLEX,fs*0.5f,cv::Scalar(125,255,255));
    cv::putText(im,"'Esc': exit",cv::Point(10,fs*120),cv::FONT_HERSHEY_SIMPLEX,fs*0.5f,cv::Scalar(125,255,255));
}

int main(int argc,char **argv){

    try{

        if (argc!=2){
            cerr<<"Usage out.bag "<<endl;
            return -1;
        }

        string output = string(argv[1]);

        rs2::context ctx;
        rs2::device_list devices = ctx.query_devices();
        if (devices.size() == 0)
            throw std::runtime_error("No device connected, please connect a RealSense device");

        // Create a shared pointer to a pipeline
        auto pipe = std::make_shared<rs2::pipeline>();

        // Start streaming with default configuration
        pipe->start();

        rs2::device device = pipe->get_active_profile().get_device();

        rs2::colorizer color_map;

        bool paused=false;
        int key=-1;
        while(key!=27){


            rs2::frameset frames = pipe->wait_for_frames(); // wait for next set of frames from the camera
            rs2::frame depth = color_map.process(frames.get_depth_frame()); // Find and colorize the depth data

            const int w = depth.as<rs2::video_frame>().get_width();
            const int h = depth.as<rs2::video_frame>().get_height();
            cv::Mat in_depth = cv::Mat(cv::Size(w, h), CV_8UC3, (void *)depth.get_data(), cv::Mat::AUTO_STEP).clone();

            if (device.as<rs2::recorder>())
            {
                if(!paused)
                    cv::circle(in_depth,cv::Point(30,30),10,cv::Scalar(0,0,255),-1);
                else{
                    cv::line(in_depth,cv::Point(20,20),cv::Point(20,30),cv::Scalar(0,0,255),3);
                    cv::line(in_depth,cv::Point(30,20),cv::Point(30,30),cv::Scalar(0,0,255),3);
                }
            }

            printHelp(in_depth);
            cv::imshow("Depth", in_depth);

            key=cv::waitKey(10);
            if (key=='s')
            {
                if (!device.as<rs2::recorder>())
                {
                    pipe->stop(); // Stop the pipeline with the default configuration
                    pipe = std::make_shared<rs2::pipeline>();
                    rs2::config cfg; // Declare a new configuration
                    cfg.enable_record_to_file(output);
                    cfg.enable_stream(RS2_STREAM_DEPTH);
                    cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8);
                    pipe->start(cfg); //File will be opened at this point
                    device = pipe->get_active_profile().get_device();
                }
                else
                {
                    pipe->stop(); // Stop the pipeline that holds the file and the recorder
                    pipe = std::make_shared<rs2::pipeline>(); //Reset the shared pointer with a new pipeline
                    pipe->start(); // Resume streaming with default configuration
                    device = pipe->get_active_profile().get_device();
                }
            }

            if (key=='p')
            {
                if (device.as<rs2::recorder>())
                {
                    if(!paused)
                    {
                        device.as<rs2::recorder>().pause();
                        paused=true;
                    }
                    else
                    {
                        device.as<rs2::recorder>().resume();
                        paused=false;
                    }
                }
            }
        }

    }catch(std::exception &ex){
        cout<<ex.what()<<endl;
    }

}
