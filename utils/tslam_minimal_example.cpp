/**
This file is part of TSLAM and distributed under the terms of the GPLv3.

Copyright (C) 2023 Andrea Settimi <andrea dot settimi at epfl dot ch>, Hong-Bin Yang <hong dash bin dot yang at epfl dot ch> (IBOIS, EPFL)

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Andrea Settimi and Hong-Bin Yang ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied Andrea Settimi and Hong-Bin Yang.
*/
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