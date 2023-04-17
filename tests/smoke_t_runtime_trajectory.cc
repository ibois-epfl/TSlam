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

       tslam::TSlam *Slam = new tslam::TSlam;
//        tslam::ImageParams imageParams;
//        tslam::Params params;
//        cv::Mat in_image;
//        cv::Mat camPose;
//
//        // init params
//        params.enableLoopClosure = false;
//        params.localizeOnly=true;
//
//        // load camera matrix and distortion coefficients
//        imageParams.readFromXMLFile(argv[2]);
//
//        auto TheMap = std::make_shared<tslam::Map>();
//        TheMap->readFromFile(cml("-map"));
//
//        VideoCapture cap("chaplin.mp4");
//        CHECK(cap.isOpened());
//
//        while(true){
//            cap >> in_image;
//            if(in_image.empty()) break;
//
//            // input image preprocessing
//            // image resize (if required)
//            if (needResize){
//                in_image = resize(in_image, cv::Size(camW, camH));
//            }
//
//            // image undistortion (if required)
//            if(undistort){
//                cv::remap(in_image,auxImage,undistMap[0],undistMap[1],cv::INTER_CUBIC);
//                in_image = auxImage;
//                imageParams.Distorsion.setTo(cv::Scalar::all(0));
//            }
//
//            camPose = Slam->process(in_image, imageParams, currentFrameIndex);
//        }

        CHECK(true);
    }

}