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


}
