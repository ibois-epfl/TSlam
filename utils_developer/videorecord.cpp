#include <opencv2/highgui.hpp>
#include <iostream>
#include "basictypes/cvversioning.h"
using namespace  std;

int main(int argc,char **argv){
    try{
        cv::VideoCapture vcap(1);
        cv::VideoWriter videoout;

        cv::Mat image;
        while(image.empty())
              vcap>>image;

        char k=0;
        int nf=0;
        while(k!=27){
cerr<<nf++<<" ";
            if(!videoout.isOpened())
                videoout.open( "video.mp4", CV_FOURCC('X', '2', '6', '4'), 30,image.size(), image.channels()!=1);
            cv::imshow("image",image);
            videoout<<image;
            vcap>>image;
            k=cv::waitKey(4);
        }


    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }


}
