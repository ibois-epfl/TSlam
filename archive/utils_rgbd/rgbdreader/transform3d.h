#ifndef TRANSFORM3D_H
#define TRANSFORM3D_H
#include <opencv2/core.hpp>

class Transform3d
{
    cv::Matx44d T;
    cv::Matx33d R;
    cv::Vec3d t;
    cv::Vec3d r;

public:
    cv::Vec3d  operator()(const cv::Vec3d &input);
    cv::Vec3f   operator()(const cv::Vec3f &input);
    cv::Point3f  operator()(const cv::Point3f &input);
    cv::Point3d  operator()(const cv::Point3d &input);
    Transform3d();
    Transform3d(cv::Matx44d input);
    Transform3d(cv::Mat input);
    void set_R(cv::Mat in_R);

    cv::Matx31d get_r();
    void set_r(cv::Vec3d in_r);

    cv::Matx31d get_t();
    void set_t(cv::Vec3d in_t);

    cv::Matx33d get_R();
    cv::Matx44d get_T(); 
};

#endif // TRANSFORM3D_H
