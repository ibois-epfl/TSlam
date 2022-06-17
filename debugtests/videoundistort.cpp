#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "reslam.h"
int main(int argc,char **argv){

    if(argc!=4){cerr<<"Usage invideo incamera outvideo"<<endl;return -1;}

    reslam::ImageParams image_params;
    image_params.readFromXMLFile(argv[2]);
    cv::VideoCapture vcap(argv[1]);

    vector<cv::Mat> undistMap;
    undistMap.resize(2);
    cv::initUndistortRectifyMap(image_params.CameraMatrix,image_params.Distorsion,cv::Mat(),cv::Mat(),image_params.CamSize,CV_32FC1,undistMap[0],undistMap[1]);

    cv::VideoWriter outv;

    while(vcap.grab()){
        cv::Mat  image,image2;
        vcap.retrieve(image);

        cv::remap(image,image2,undistMap[0],undistMap[1],cv::INTER_CUBIC);
        if( outv.isOpened()==false)
            outv.open(argv[3], CV_FOURCC('X', '2', '6', '4'), 30,image.size()  , image.channels()!=1);
        outv<<image2;
        cout<<vcap.get(CV_CAP_PROP_FRAME_COUNT)<<" "<<vcap.get(CV_CAP_PROP_POS_FRAMES)<<endl;

    }
}
