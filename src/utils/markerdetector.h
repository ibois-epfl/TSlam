/**
* This file is part of  UCOSLAM
*
* Copyright (C) 2020 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)
*
* UCOSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* UCOSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with UCOSLAM. If not, see <http://wwmap->gnu.org/licenses/>.
*/

#ifndef UCOSLAM_MarkerDetector_H
#define UCOSLAM_MarkerDetector_H

#include <memory>
#include <opencv2/core/core.hpp>
#include "reslamtypes.h"
namespace aruco {
class MarkerDetector;
}

namespace  reslam {


//minimal information that must be provided by a marker detector
struct MarkerDetection{
    uint32_t id;//unique marker id
    std::vector<cv::Point3f> points3d;//three dimentional points of the marker wrt its center
    std::vector<cv::Point2f> corners;//original corners in the image
    std::string info;//optional info about the marker
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

}
#endif
