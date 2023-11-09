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
#include "input_reader.h"
#include "dir_reader.h"
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <chrono>
#include <iostream>
#include "basictypes/cvversioning.h"
InputReader::InputReader( )
{
}
void InputReader::open(std::string path,bool emulateRealTime){
    _emulateRealTime=emulateRealTime;
    vcap.open(path);
    if(!vcap.isOpened()){//try dir
        files=DirReader::read(path,"",DirReader::Params(true));
        imgIdx=0;
    }
    isLive=false;
}
void InputReader::open(int cameraIdx){
    vcap.open(cameraIdx);
    isLive=true;
    imgIdx=0;
    videoTimeBetweenFrame=std::chrono::milliseconds(0);
}

bool InputReader::isOpened()const{
    if(vcap.isOpened())return true;
    if(files.size()!=0)return true;
    return false;
}
void InputReader:: set(int v,double val){
    if( vcap.isOpened()){
            vcap.set(v,val);
        }
    else{//images
        if(v==CV_CAP_PROP_POS_FRAMES && val<files.size()) imgIdx=val;
    }
}
double InputReader:: get(int v){

    if( vcap.isOpened()){
        if( isLive && v==CV_CAP_PROP_POS_FRAMES){
            return imgIdx;
        }

             else return vcap.get(CV_CAP_PROP_POS_FRAMES);
        }
    else{//images
        if(v==CV_CAP_PROP_POS_FRAMES) return imgIdx;
    }

    throw std::runtime_error("InputReader::get Should not get here");
}
int InputReader::getNextFrameIndex(  ){

    if(!isOpened()) return -1;
    if(vcap.isOpened()){
        if (isLive) return imgIdx;
        else return  int(vcap.get(CV_CAP_PROP_POS_FRAMES));
    }
    else{
        return imgIdx;
    }
}

InputReader & 	InputReader::operator>> (cv::Mat &image){
    if ( grab())
        retrieve(image);
    else image=cv::Mat();
    return *this;
}

bool InputReader::grab(){
    if(vcap.isOpened()){
        if(isLive)imgIdx++;
        else if(_emulateRealTime){
            //wait to deliver at the appropriate speed
            std::chrono::milliseconds waitTime=std::chrono::milliseconds(0);
            if(videoTimeBetweenFrame== std::chrono::milliseconds(0)){//first time here
                float fps=vcap.get(CV_CAP_PROP_FPS);
                videoTimeBetweenFrame =  std::chrono::milliseconds(  int(1000./fps) );
                lastCapture=std::chrono::high_resolution_clock::now();
            }
            else{
                auto now=std::chrono::high_resolution_clock::now();
                auto elapsedTime=std::chrono::duration_cast<std::chrono::milliseconds>( now-lastCapture);
                if( elapsedTime< videoTimeBetweenFrame)
                    waitTime=videoTimeBetweenFrame-elapsedTime;
                lastCapture=now;
            }
            std::this_thread::sleep_for(waitTime);
        }
        return vcap.grab();
    }

    do{
     fileImage=cv::imread(files[imgIdx++]);
    }while(fileImage.empty() &&imgIdx<files.size());

    return !fileImage.empty();
}

void InputReader::retrieve(cv::Mat &im){
    if(vcap.isOpened()){
        vcap.retrieve(im);
    }
    else fileImage.copyTo(im);

}
