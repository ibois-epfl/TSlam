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