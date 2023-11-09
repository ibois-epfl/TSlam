/**
This file is part of TSLAM and distributed under the terms of the GPLv3.

The modified version: Copyright (C) 2023 Andrea Settimi <andrea dot settimi at epfl dot ch>, Hong-Bin Yang <hong dash bin dot yang at epfl dot ch> (IBOIS, EPFL)
The original project: Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas, MODIFIED BY Andrea Settimi and Hong-Bin Yang ''AS IS'' AND ANY EXPRESS OR IMPLIED
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
or implied, of Rafael Muñoz Salinas.
*/
#ifndef _StereoRectify_H
#define _StereoRectify_H
#include <string>
#ifdef STANDALONE
    #define TSLAM_API
#else
    #include "imageparams.h"
    #include "tslam_exports.h"
#endif
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <thread>
namespace tslam{

/** Stereo image rectification
 *
 * Reads the stereo config from a YML and then rectifies the images so that they can be passed to UcoSLAM.
 *
 * The file must have been generated with with the  program in utils/tslam_stereocalibrate
 *
 * The file must have the matrices for stereo rectification ( M1,M2,D1...) and image dimensions (image_width,image_height)
 */
class TSLAM_API StereoRectify{
public:


    //reads the data from XML file.
    void readFromXMLFile(const std::string &path){ readFromXMLFile_(path);}

    //does the work.
    void rectify( const cv::Mat &left,const cv::Mat &right){rectify_(left,right);}
    cv::Mat rectify( const cv::Mat &img,int idx){return rectify_(img,idx);}
    //returns the image params (of the rectified images) needed to call UcoSLAM
#ifndef STANDALONE
    inline ImageParams getImageParams()const{return imgParams;}
#endif
    //returns the left rectified image
    inline cv::Mat & getLeft() {return leftRect;}
    //returns the right rectified image
    inline cv::Mat &getRight() {return rightRect;}

private:
    cv::Mat leftRect,rightRect;
    cv::Mat mapx[2],mapy[2];
#ifndef STANDALONE
    ImageParams imgParams;
#endif


    //reads the data from XML file.
    void  readFromXMLFile_(const std::string &filePath){
        cv::FileStorage fs(filePath, cv::FileStorage::READ);
        if(!fs.isOpened()) throw std::runtime_error(std::string(__FILE__)+" could not open file:"+filePath);

        for(int i=0;i<2;i++){
            mapx[i]=cv::Mat();
            mapy[i]=cv::Mat();
        }
        cv::Mat M1,M2,D1,D2,R,T,R1,R2,P1,P2 ;
        int w = -1, h = -1;
        fs["image_width"]>>w;
        fs["image_height"]>>h;
        fs["M1"]>>M1;
        fs["M2"]>>M2;
        fs["D1"]>>D1;
        fs["D2"]>>D2;
        fs["T"]>>T;
        fs["R1"]>>R1;
        fs["R2"]>>R2;
        fs["P1"]>>P1;
        fs["P2"]>>P2;
        //consider only the 3x3 part of P1 and P2 as the new camera matrix
        cv::Mat ncm[2];
        ncm[0]=P1.rowRange(0,3).colRange(0,3);
        ncm[1]=P2.rowRange(0,3).colRange(0,3);


        if(w==-1 || h==-1)  throw std::runtime_error("StereoRectify::readFromXMLFile_ image_width or image_height not found in YML file");


        cv::initUndistortRectifyMap(M1,D1,R1,ncm[0],cv::Size( w,h),CV_16SC2,mapx[0],mapy[0]);
        cv::initUndistortRectifyMap(M2,D2,R2,ncm[1],cv::Size( w,h),CV_16SC2,mapx[1],mapy[1]);
#ifndef STANDALONE
        imgParams.CamSize=cv::Size(w,h);
        ncm[0].convertTo(imgParams.CameraMatrix,CV_32F);
        imgParams.Distorsion=cv::Mat::zeros(1,5,CV_32F);
        imgParams.bl=cv::norm(T);
#endif
    }


    void  rectify_( const cv::Mat &left,const cv::Mat &right){

        std::thread LeftThread=std::thread([&](){cv::remap(left,leftRect,mapx[0],mapy[0],cv::INTER_LINEAR);});
        std::thread RightThread=std::thread([&](){cv::remap(right,rightRect,mapx[1],mapy[1],cv::INTER_LINEAR);});
        LeftThread.join();
        RightThread.join();


    }
    cv::Mat rectify_( const cv::Mat &img,int idx){
        if(idx==0){
            cv::remap(img,leftRect,mapx[0],mapy[0],cv::INTER_LINEAR);
            return leftRect;
        }
        else  {
            cv::remap(img,rightRect,mapx[1],mapy[1],cv::INTER_LINEAR);
            return rightRect;
        }
    }

};



};

#endif
