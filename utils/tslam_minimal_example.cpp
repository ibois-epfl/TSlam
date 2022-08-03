#include "opencv2/opencv.hpp"
#include "tslam.h"

using namespace cv;
int main(){
    // initialize slam
    auto mMap = std::make_shared<tslam::Map>();
    mMap->readFromFile("../../example/example.map");
    
    tslam::Params params;
    params.isInstancing = true;

    string vocPath = "../../orb.fbow";

    tslam::TSlam *slam = new tslam::TSlam;
    slam->setParams(mMap, params, vocPath);

    cv::Mat camPose;

    // camera params
    tslam::ImageParams imageParams;
    imageParams.readFromXMLFile("../../example/calibration_webcam.yml");

    /* read video from camera 0 */
    // VideoCapture cap(0); 
    
    /* read from video sequence */
    VideoCapture cap("../../example/video.mp4");

    if(!cap.isOpened()){
	    cout << "Error opening video stream" << endl;
	    return -1;
    }

    Mat frame;
    int frameIdx = -1;
    while(true){
        cap >> frame; frameIdx++;
        if(frame.empty()) break;

        camPose=slam->process(frame, imageParams, frameIdx);
        cout << "Frame " << frameIdx << ": " << (camPose.empty()?"not tracked":"tracked") << ", Camera Pose: " <<endl;
        cout << camPose << endl;

        imshow("Frame", frame);
        char c = (char)waitKey(1);
        if(c == 27) break;
    }

    cap.release();
    destroyAllWindows();
    
    return 0;
}