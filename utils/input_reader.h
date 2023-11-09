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
#ifndef INPUTREADER_H
#define INPUTREADER_H
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
/**Class to read either from video, camera or a directory of images
 */

class InputReader
{
public:
    InputReader( );

    void open(std::string path,bool emulateRealTime=false);
    void open(int cameraIdx);
    bool isOpened()const;
    bool grab();
    void retrieve(cv::Mat &im);
    void set(int v,double val);
    double get(int v);
    virtual InputReader & 	operator>> (cv::Mat &image);
    int cIndexLive=0;
    int getNextFrameIndex( ) ;
    private:
    cv::VideoCapture vcap;
    std::vector<std::string> files;
    int imgIdx=0;
    cv::Mat fileImage;
    bool isLive=false;

private:
    bool _emulateRealTime=false;
    std::chrono::high_resolution_clock::time_point lastCapture;
     std::chrono::milliseconds videoTimeBetweenFrame= std::chrono::milliseconds(0);

};

#endif // INPUTREADER_H
