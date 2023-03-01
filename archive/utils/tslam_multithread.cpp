#include "opencv2/opencv.hpp"
#include "tslam.h"
#include "mapviewer.h"
#include <thread>
#include <mutex>

using namespace std;
using namespace cv;
int main(){
    cv::Mat camPoseBuf[2];
    cv::Mat frameBuf[2];
    
    bool isLocked[2] = {0, 0};

    int procBufFlag = 0;
    int renderBufFlag = 0;

    int frameIdx = -1;

    // initialize slam
    auto mMap = std::make_shared<tslam::Map>();
    mMap->readFromFile("long_new_param_comb.map");
    
    tslam::Params params;
    params.isInstancing = true;

    string vocPath = "../../orb.fbow";

    tslam::TSlam *slam = new tslam::TSlam;
    slam->setParams(mMap, params, vocPath);

    // camera params
    tslam::ImageParams imageParams;
    imageParams.readFromXMLFile("../../example/calibration_webcam.yml");

    tslam::MapViewer mapViewer;

    /* read video from camera 0 */
    VideoCapture cap(0); 
    
    /* read from video sequence */
    // VideoCapture cap("../../example/video.mp4");

    if(!cap.isOpened()){
	    cerr << "Error opening video stream" << endl;
	    return -1;
    }

    // std::mutex myMutex[2];

    auto procLambda = [&](cv::Mat *camPose, cv::Mat *frame, tslam::ImageParams *imageParams, int frameIdx, int* procBufFlag, int* renderBufFlag){
        cerr << "Start processing..." << "(buf " << *procBufFlag << ")" << endl;
        *camPose = slam->process(*frame, *imageParams, frameIdx);
        isLocked[*procBufFlag] = false;
        *procBufFlag = !*procBufFlag;
        cerr << "Done processing..." << endl;
    };

    auto renderLambda = [&](bool *breakFlag, shared_ptr<tslam::Map> mMap, cv::Mat *frame, cv::Mat *camPose, int frameIdx, int* procBufFlag, int* renderBufFlag, bool* isRendering){
        cerr << "Start rendering..." << "(buf " << *renderBufFlag << ")" << endl;
        char c = mapViewer.show(mMap, *frame, *camPose, "#" + std::to_string(frameIdx), frameIdx);
        if(c == 27 || c=='q') *breakFlag = true;

        cout << "Frame " << frameIdx << ": " << ((*camPose).empty()?"not tracked":"tracked") << ", Camera Pose: " <<endl;
        cout << *camPose << endl;

        isLocked[*renderBufFlag] = false;
        *renderBufFlag = !*renderBufFlag;
        cerr << "Done rendering!" << endl;
        *isRendering = false;
    };

    cv::Mat frame;
    
    int whileCnt = 0;
    bool breakFlag = false;
    bool isRendering = false;
    while(true){
        cerr << "In " << ++whileCnt << " loop:" << endl;
        if(breakFlag) break;
        cap >> frame; if(frame.empty()) break;

        std::thread procThread, procRender;

        bool procThreadStarted = false;
        // proc thread
        if(isLocked[procBufFlag]) { cerr << "proc buf (" << procBufFlag << ") is locked, skip frame." << endl; continue; }
        else{
            cerr << "proc buf (" << procBufFlag << ") is not locked, process." << endl;
            frameBuf[procBufFlag] = frame; frameIdx++;
            isLocked[procBufFlag] = true;
            procThreadStarted = true;
            procThread = std::thread (procLambda, &camPoseBuf[procBufFlag], &frameBuf[procBufFlag], &imageParams, frameIdx, &procBufFlag, &renderBufFlag);
        }

        if(isLocked[renderBufFlag]){
            cerr << "render buf (" << renderBufFlag << ") is locked, wait." << endl;
            if(procThreadStarted) procThread.join();
        } else {
            cerr << "render buf (" << renderBufFlag << ") is not locked, render." << endl;
            if(procThreadStarted) procThread.join();
        }

        if(!isRendering){
            isRendering = true;
            isLocked[renderBufFlag] = true;
            procRender = std::thread (renderLambda, &breakFlag, mMap, &frameBuf[renderBufFlag], &camPoseBuf[renderBufFlag], frameIdx, &procBufFlag, &renderBufFlag, &isRendering);
            procRender.detach();
        }
    }

    cap.release();
    destroyAllWindows();
    
    return 0;
}