
#include <opencv2/core.hpp>
#include <aruco/aruco.h>
#include "basictypes/misc.h"

class StereoCalibration
{
public:
    StereoCalibration();
    ~StereoCalibration();

    void setParams(const float markerSize, const aruco::MarkerMap& markerMap);
    void setParams(const float markerSize, const aruco::MarkerMap& markerMap, const aruco::MarkerDetector& markerDetector);

    void calibration(const std::vector<std::string>& imagelist, bool displayDetectedMarkers=false);

    double stereo_solve(const std::vector<std::vector<std::vector<cv::Point3f>>>& calib3d,
                        const std::vector<std::vector<std::vector<cv::Point2f>>>& calib2d,
                        const std::vector<std::vector<cv::Mat>>& cameraPose,
                        cv::Mat& r_io, cv::Mat& t_io);

    cv::Mat getRTMatrix() const{return ucoslam::getRTMatrix(_rvec,_tvec);}
    cv::Mat getRvec() const {return _rvec;}
    cv::Mat getTvec() const {return _tvec;}
    std::vector<aruco::CameraParameters> _camParams;


private:
    float _msize;
    aruco::MarkerMap _mmap;
    aruco::MarkerDetector _mdetector;

    cv::Mat _rvec, _tvec; //Extrinsic
};
