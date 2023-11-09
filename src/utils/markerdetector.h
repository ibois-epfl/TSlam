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

#ifndef TSLAM_MarkerDetector_H
#define TSLAM_MarkerDetector_H

#include <memory>
#include <opencv2/core/core.hpp>
#include "tslamtypes.h"
#include "stag/Stag.h"
namespace aruco {
class MarkerDetector;
}

namespace  tslam {


//minimal information that must be provided by a marker detector
struct MarkerDetection{
    uint32_t id;
    std::vector<cv::Point3f> points3d;//three dimentional points of the marker wrt its center
    std::vector<cv::Point2f> corners;//original corners in the image
    std::string info;
};

//Base class for marker detectors
class MarkerDetector{
public:
    //returns the marker observations in the image passed
    //your reimplementation needs to fill the fields of the base class

    virtual std::vector<MarkerDetection> detect(const cv::Mat &Image)=0;
    virtual std::string getName()const=0;
    virtual void toStream(std::ostream &str)const{}
    virtual void fromStream(std::istream &str){}
    virtual void setParams(const Params &p){}
};


//Aruco marker detector
class ArucoMarkerDetector:public MarkerDetector{
    Params _p;
    std::shared_ptr<aruco::MarkerDetector> _mdetector;
public:
    ArucoMarkerDetector();
    ArucoMarkerDetector(const Params &p);
    void setParams(const Params &p)override;
    std::string getName()const override{return "aruco";}
    std::vector<MarkerDetection> detect(const cv::Mat &Image)override;
    void toStream(std::ostream &str)const override;
    void fromStream(std::istream &str)override;

};

//STag detector
class STagMarkerDetector:public MarkerDetector{
    Stag _stag;
    Params _p;
    std::shared_ptr<aruco::MarkerDetector> _mdetector;
public:
    STagMarkerDetector(int libraryHD=11, int errorCorrection=7, bool inKeepLogs=false);
    STagMarkerDetector(const Params &p, int libraryHD=11, int errorCorrection=7, bool inKeepLogs=false);
    void setParams(const Params &p)override;
    std::string getName()const override{return "stag";}
    std::vector<MarkerDetection> detect(const cv::Mat &Image)override;
    void toStream(std::ostream &str)const override;
    void fromStream(std::istream &str)override;
};

}
#endif
