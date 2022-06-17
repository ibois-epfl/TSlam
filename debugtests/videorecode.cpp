#include <opencv2/highgui.hpp>
#include <iostream>
#include "reslam.h"
int main(int argc,char **argv){

    cv::VideoCapture vcap(argv[1]);

    cv::VideoWriter outv;

    while(vcap.grab()){
        cv::Mat  image;
        vcap.retrieve(image);
        if( outv.isOpened()==false)
            outv.open(argv[2], CV_FOURCC('X', '2', '6', '4'), 30,image.size()  , image.channels()!=1);
        outv<<image;
        cout<<vcap.get(CV_CAP_PROP_FRAME_COUNT)<<" "<<vcap.get(CV_CAP_PROP_POS_FRAMES)<<endl;

    }
}
