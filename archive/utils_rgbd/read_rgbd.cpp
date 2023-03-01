#include "rgbdreaderfactory.h"
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
void print_usage(){
    cout<<"Usage: rgbdreader <path-to-data-folder> <dataset-flag>"<<endl;
    cout<<"\t dataset flags: oni sun3d nyu redwood"<<endl;
}

int main(int argc, char* argv[]){
    if(argc<2){
        print_usage();
        return -1;
    }
    shared_ptr<RGBDReader> data_reader=RGBDReaderFactory::getReader(argv[2],argv[1]);
    cv::Mat depth,color;
    size_t frame_num=0;
    while(data_reader->getNextFrame(depth,color)){
        cv::putText(color,to_string(frame_num),cv::Point(100,100),cv::FONT_HERSHEY_SIMPLEX,3,cv::Scalar(255,255,255));
        cv::imshow("color",color);
        cv::imshow("depth",depth*30.0);
        char c=cv::waitKey();
        if(c=='q')
            break;
        frame_num++;
    }
    return 0;
}
