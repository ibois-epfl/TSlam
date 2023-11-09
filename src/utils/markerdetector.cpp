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
#include "markerdetector.h"
#include "aruco/markerdetector.h"
#include "optimization/ippe.h"

namespace tslam {


ArucoMarkerDetector::ArucoMarkerDetector(){
    _mdetector=std::make_shared<aruco::MarkerDetector>();
}
ArucoMarkerDetector:: ArucoMarkerDetector(const Params &p){
    _mdetector=std::make_shared<aruco::MarkerDetector>();
    setParams(p);
}


void ArucoMarkerDetector::setParams(const Params &p){
    aruco::MarkerDetector::Params aruco_DetectorParams;
    _p=p;
    aruco_DetectorParams.setDetectionMode(  aruco::MarkerDetector::Params::getDetectionModeFromString(_p.aruco_DetectionMode ),_p.aruco_minMarkerSize);
    aruco_DetectorParams.dictionary=_p.aruco_Dictionary;
    aruco_DetectorParams.setCornerRefinementMethod( aruco::MarkerDetector::Params::getCornerRefinementMethodFromString(_p.aruco_CornerRefimentMethod) );
    _mdetector->setParameters(aruco_DetectorParams);

}

std::vector<MarkerDetection> ArucoMarkerDetector::detect(const cv::Mat &Image){
    std::vector<MarkerDetection> markers;

    for(const auto&m:_mdetector->detect(Image)){
        MarkerDetection uslm_marker;
        uslm_marker.id=m.id;
        uslm_marker.points3d=m.get3DPoints(_p.aruco_markerSize);
        uslm_marker.corners=m;
        uslm_marker.info=m.dict_info;
        markers.push_back(uslm_marker);
    }
    return markers;
}
void ArucoMarkerDetector:: toStream(std::ostream &str)const{
    _mdetector->toStream(str);
}
void ArucoMarkerDetector:: fromStream(std::istream &str){
    _mdetector->fromStream(str);
}


//////////////////////////////////
// STagMarkerDetector interface //
//////////////////////////////////
STagMarkerDetector::STagMarkerDetector(int libraryHD, int errorCorrection, bool inKeepLogs){
    _mdetector=std::make_shared<aruco::MarkerDetector>();
    _stag = Stag(libraryHD, errorCorrection, inKeepLogs);
}
STagMarkerDetector::STagMarkerDetector(const Params &p, int libraryHD, int errorCorrection, bool inKeepLogs){
    _mdetector=std::make_shared<aruco::MarkerDetector>();
    _stag = Stag(libraryHD, errorCorrection, inKeepLogs);
    setParams(p);
}

void STagMarkerDetector::setParams(const Params &p){
    aruco::MarkerDetector::Params aruco_DetectorParams;
    _p=p;
    aruco_DetectorParams.setDetectionMode( aruco::MarkerDetector::Params::getDetectionModeFromString(_p.aruco_DetectionMode ),_p.aruco_minMarkerSize);
    aruco_DetectorParams.dictionary=_p.aruco_Dictionary;
    aruco_DetectorParams.setCornerRefinementMethod( aruco::MarkerDetector::Params::getCornerRefinementMethodFromString(_p.aruco_CornerRefimentMethod) );
    _mdetector->setParameters(aruco_DetectorParams);

}

std::vector<MarkerDetection> STagMarkerDetector::detect(const cv::Mat &Image){
    std::vector<MarkerDetection> markers;
    _stag.detectMarkers(Image);

    for(const auto m : _stag.getMarkerList()){
        MarkerDetection uslm_marker;
        // ID should be no problem
        uslm_marker.id=m.id;

        // Just leave it
        float halfSize = _p.aruco_markerSize / 2.f;
        uslm_marker.points3d={cv::Point3f(-halfSize, halfSize, 0),cv::Point3f(halfSize, halfSize, 0),cv::Point3f(halfSize,-halfSize, 0),cv::Point3f(-halfSize, -halfSize, 0)};

        // here: vector<cv::Point2f> corners
        // stag: vector<cv::Point2d> corners
        std::vector<cv::Point2f> corners;
        for (auto c: m.corners){
            corners.push_back(cv::Point2f(c.x, c.y));
        }
        uslm_marker.corners=corners;

        // The info seems not to be used
        uslm_marker.info="m.dict_info";

        markers.push_back(uslm_marker);
    }
    return markers;
}
void STagMarkerDetector:: toStream(std::ostream &str)const{
    _mdetector->toStream(str);
}
void STagMarkerDetector:: fromStream(std::istream &str){
    _mdetector->fromStream(str);
}


}
