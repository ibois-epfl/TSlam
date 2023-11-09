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
#ifdef CMAKE_PROJECT_SOURCE_DIR  // this should be defined in CMAKE with -DCMAKE_PROJECT_SOURCE_DIR
#define PROJECT_DIR CMAKE_PROJECT_SOURCE_DIR
#else
#define PROJECT_DIR "../"
#endif

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "tslam.h"
#include <iostream>
#include <opencv2/opencv.hpp>

TEST_SUITE("Smoke_tslam_tests")
{
    TEST_CASE("Smoke_tslam_test")
    {
        // call the runtime from here
        // TODO: put everything needed for tests in src/tests/data

        string projectRoot = PROJECT_DIR;

        cv::theRNG().state = 0;
        srand(0);

        int badPredCount = 0;

        tslam::TSlam *Slam = new tslam::TSlam();
        tslam::ImageParams imageParams;
        tslam::Params params;
        cv::Mat inImage;
        cv::Mat auxImage;

        // init params
        params.enableLoopClosure = false;
        params.localizeOnly = true;

        // load camera matrix and distortion coefficients
        string loadPath = projectRoot + "/tests/";
        cout << "Testing loadPath: " << loadPath << endl;

        imageParams.readFromXMLFile(loadPath + "example_calibration.yml");
        int camW = int(imageParams.CamSize.width);
        int camH = int(imageParams.CamSize.height);

        vector<cv::Mat> undistMap;
        undistMap.resize(2);
        cv::initUndistortRectifyMap(imageParams.CameraMatrix, imageParams.Distorsion, cv::Mat(), cv::Mat(),
                                    imageParams.CamSize, CV_32FC1, undistMap[0], undistMap[1]);
        imageParams.Distorsion.setTo(cv::Scalar::all(0));

        auto TheMap = std::make_shared<tslam::Map>();
        TheMap->readFromFile(loadPath + "example.map");

        Slam->setParams(TheMap, params, projectRoot + "/assets/voc/orb.fbow");

        // open video
        cv::VideoCapture cap(loadPath + "example_video.mp4");
        REQUIRE(cap.isOpened());

        int currentFrameIndex = 0;

        vector<cv::Mat> predCamPose;
        cout << "Predicting camera poses..." << flush;
        while (true) {
            // skip 2 frames
            cap >> inImage;
            cap >> inImage;
            cap >> inImage;

            if (currentFrameIndex % 10 == 0) {
                cout << "." << flush;
            }

            if (inImage.empty() || currentFrameIndex > 100) {
                break;
            }

            // input image preprocessing
            // image resize
            cv::resize(inImage, auxImage, cv::Size(camW, camH));
            inImage = auxImage;

            // image undistortion
            cv::remap(inImage, auxImage, undistMap[0], undistMap[1], cv::INTER_CUBIC);
            inImage = auxImage;

//            cv::imshow("inImage", inImage);
//            cv::waitKey(1);

            cv::Mat camPose = Slam->process(inImage, imageParams, currentFrameIndex++);
            if(camPose.empty()){
                badPredCount++;
                CHECK(badPredCount < 50);
                camPose = cv::Mat::zeros(4,4,CV_32F);
            }
            predCamPose.push_back(camPose);
        }
        cout << "Done!" << endl;

//#ifdef GEN_GT
//
//        /* This part is for generating the Ground Truth */
//        // write result
//        ofstream groundTruthTxt;
//        groundTruthTxt.open (loadPath + "example_test_gt.txt");
//        for(auto camPose: predCamPose){
//            for(int i = 0 ; i < 4 ; i ++){
//                for(int j = 0 ; j < 4 ; j ++){
//                    groundTruthTxt << camPose.at<float>(i,j) << " ";
//                }
//            }
//            groundTruthTxt << endl;
//        }
//        groundTruthTxt.close();
//
//#endif

        // read result
        ifstream gtTxt;
        gtTxt.open(loadPath + "example_pose_gt.txt");
        vector<cv::Mat> gtCamPose;
        while(!gtTxt.eof()){
            cv::Mat camPose = cv::Mat::eye(4,4,CV_32F);
            for(int i = 0 ; i < 4 ; i ++){
                for(int j = 0 ; j < 4 ; j ++){
                    gtTxt >> camPose.at<float>(i,j);
                }
            }
            gtCamPose.push_back(camPose);
        }

        // Compare the pred camera pose with the ground truth

        cout << "Checking if the pred camera pose is close to the ground truth..." << flush;
        for(int i = 0 ; i < predCamPose.size() ; i ++){
            if (predCamPose[i].at<float>(0, 0) == 0.0f){
                badPredCount++;
            }
            cv::Mat diff = predCamPose[i] - gtCamPose[i];
            float error = cv::mean(diff)[0];
            CHECK(error < 0.1);
        }
        cout << "All good!" << endl;
        CHECK(true);
    }
}