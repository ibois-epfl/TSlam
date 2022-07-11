#include "transform3d.h"
#include <opencv2/calib3d.hpp>

Transform3d::Transform3d(){
    T=cv::Matx44d::eye();
    R=cv::Matx33d::eye();
    for(int i=0;i<3;i++)
        r[i]=t[i]=0;
}

Transform3d::Transform3d(cv::Matx44d input){
    R=cv::Matx33d::eye();
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++)
            R(i,j)=input(i,j);

        t[i]=input(i,3);
    }
    cv::Rodrigues(R,r);
    T=input;
}

Transform3d::Transform3d(cv::Mat input){
    input.convertTo(input,CV_64FC1);
    T=cv::Matx44d(input);
    R=cv::Matx33d();
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++)
            R(i,j)=T(i,j);
        t[i]=T(i,3);
    }
    cv::Rodrigues(R,r);
}

cv::Vec3d Transform3d::operator ()(const cv::Vec3d &input){
    return R*input+t;
}

cv::Point3d Transform3d::operator ()(const cv::Point3d &input){
    return R*cv::Vec3d(input)+t;
}

cv::Point3f Transform3d::operator ()(const cv::Point3f &input){
    return cv::Vec3f(R*cv::Vec3d(cv::Point3d(input))+t);
}

cv::Vec3f Transform3d::operator()(const cv::Vec3f &input){
    return R*cv::Vec3d(input)+t;
}

void Transform3d::set_R(cv::Mat in_R){
    in_R.convertTo(in_R,CV_64FC1);
    R=cv::Matx33d(in_R);
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            T(i,j)=R(i,j);
}

void Transform3d::set_t(cv::Vec3d in_t){
    t=in_t;
    for(int i=0;i<3;i++)
        T(i,3)=t(i);
}

void Transform3d::set_r(cv::Vec3d in_r){
    r=in_r;
    cv::Rodrigues(r,R);
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            T(i,j)=R(i,j);
}

cv::Matx31d Transform3d::get_t(){
    return t;
}

cv::Matx31d Transform3d::get_r(){
    return r;
}

cv::Matx44d Transform3d::get_T()
{
    return T;
}

cv::Matx33d Transform3d::get_R()
{
    return R;
}
