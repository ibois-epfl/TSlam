#include "markerdetector.h"
#include "aruco/markerdetector.h"
#include "optimization/ippe.h"

namespace reslam {


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

////////////////////////////
// STagDetector interface //
////////////////////////////

STagDetector::STagDetector(){
    _mdetector=std::make_shared<aruco::MarkerDetector>();
}
STagDetector::STagDetector(int libraryHD = 11, int errorCorrection = 7, bool inKeepLogs = false){
    _mdetector=std::make_shared<aruco::MarkerDetector>();
    _stag = Stag(libraryHD, errorCorrection, inKeepLogs);
}
STagDetector::STagDetector(const Params &p){
    _mdetector=std::make_shared<aruco::MarkerDetector>();
    setParams(p);
}

void STagDetector::setParams(const Params &p){
    aruco::MarkerDetector::Params aruco_DetectorParams;
    _p=p;
    aruco_DetectorParams.setDetectionMode( aruco::MarkerDetector::Params::getDetectionModeFromString(_p.aruco_DetectionMode ),_p.aruco_minMarkerSize);
    aruco_DetectorParams.dictionary=_p.aruco_Dictionary;
    aruco_DetectorParams.setCornerRefinementMethod( aruco::MarkerDetector::Params::getCornerRefinementMethodFromString(_p.aruco_CornerRefimentMethod) );
    _mdetector->setParameters(aruco_DetectorParams);

}

std::vector<MarkerDetection> STagDetector::detect(const cv::Mat &Image){
    std::vector<MarkerDetection> markers;
    _stag.detectMarkers(Image)

    for(const auto m : _stag.getMarkerList()){
        MarkerDetection uslm_marker;
        // ID should be no problem
        uslm_marker.id=m.id;
        
        // Just leave it
        uslm_marker.points3d=m.get3DPoints(_p.aruco_markerSize);

        // here: vector<cv::Point2f> corners
        // stag: vector<cv::Point2d> corners
        uslm_marker.corners=m.corners;

        // The info seems not to be used
        uslm_marker.info="m.dict_info";

        markers.push_back(uslm_marker);
    }
    return markers;
}
void STagDetector:: toStream(std::ostream &str)const{
    _mdetector->toStream(str);
}
void STagDetector:: fromStream(std::istream &str){
    _mdetector->fromStream(str);
}

}
