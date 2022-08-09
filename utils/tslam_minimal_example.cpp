#include "opencv2/opencv.hpp"
#include "tslam.h"
#include "mapviewer.h"
using namespace cv;
int main(){
    // initialize slam
    tslam::TSlam slam;
    slam.setMap("long_new_param_comb.map");
    slam.setVocabulary("../../orb.fbow");
    slam.setCamParams("../../example/calibration_webcam.yml");
    slam.setInstancing(true);

    cv::Mat camPose;

    auto mapViewer = new tslam::MapViewer();

    /* read video from camera 0 */
    VideoCapture cap(0); 
    
    /* read from video sequence */
    // VideoCapture cap("../../example/video.mp4");

    if(!cap.isOpened()){
	    cout << "Error opening video stream" << endl;
	    return -1;
    }

    Mat frame;
    int frameIdx = -1;
    bool isTracked;
    while(true){
        cap >> frame; frameIdx++;
        if(frame.empty()) break;

        isTracked = slam.process(frame, camPose);
        cout << "Frame #" << frameIdx << ": " << (isTracked?"tracked":"not tracked") << ", Camera Pose: " <<endl;
        cout << slam.getLastTrackedCamPose() << endl;

        char c = mapViewer->show(
            slam.map,
            frame,
            slam.getLastTrackedCamPose(),
            "#" + std::to_string(frameIdx),
            frameIdx
        );
        if(c == 27 || c=='q') break;
    }

    cap.release();
    destroyAllWindows();
    
    return 0;
}