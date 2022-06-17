#include "pointcloud.h"
#include <opencv2/imgproc.hpp>
using namespace std;

PointCloud::PointCloud()
{

}

void PointCloud::clear(){
    points.clear();
    colors.clear();
    normals.clear();
}

bool PointCloud::empty(){
    return points.empty();
}

void PointCloud::reverseNormals(){
    for(cv::Vec3f &normal:normals)
        normal=-normal;
}

size_t PointCloud::size() const{
    return points.size();
}

PointCloud::PointCloud(const vector<cv::Point3f> &in_points, const vector<cv::Vec3b> &in_colors){
    if(in_points.size()!=in_colors.size())
        throw runtime_error("There should be the same number of input points and input point colors!");
    points=in_points;
    colors=in_colors;
}

PointCloud PointCloud::transform(const cv::Mat &T) const{
    PointCloud out_cloud=*this;
    cv::Matx33f R;
    cv::Point3f t;
    get_Rt(T, R, t);
    for(size_t i=0;i<points.size();i++){
        cv::Point3f &p=out_cloud.points[i];
        p=R*p+t;
    }
    for(size_t i=0;i<normals.size();i++){
        cv::Vec3f &n=out_cloud.normals[i];
        n=cv::normalize(R*n);
    }

    return out_cloud;
}

PointCloud PointCloud::operator + (const PointCloud& in_pc) const{
    PointCloud out_pc;
    out_pc.points=points;
    out_pc.points.insert(out_pc.points.end(),in_pc.points.begin(),in_pc.points.end());
    out_pc.colors=colors;
    out_pc.colors.insert(out_pc.colors.end(),in_pc.colors.begin(),in_pc.colors.end());
    return out_pc;
}

void PointCloud::get_Rt(const cv::Mat &T, cv::Matx33f &R, cv::Point3f &t){
    cv::Mat T_32;
    T.convertTo(T_32,CV_32FC1);
    R=cv::Matx33f(T_32(cv::Range(0,3),cv::Range(0,3)));
    t=cv::Vec3f(T_32(cv::Range(0,3),cv::Range(3,4)));
}

PointCloud PointCloud::combineClouds(const vector<PointCloud> &in_clouds){
    PointCloud out_cloud;
    for(const PointCloud &in_cloud:in_clouds){
        out_cloud.points.insert(out_cloud.points.end(),in_cloud.points.begin(),in_cloud.points.end());
        out_cloud.colors.insert(out_cloud.colors.end(),in_cloud.colors.begin(),in_cloud.colors.end());
        out_cloud.normals.insert(out_cloud.normals.end(),in_cloud.normals.begin(),in_cloud.normals.end());
    }
    return out_cloud;
}

std::vector<PointCloud> PointCloud::transformClouds(const std::vector<PointCloud>& in_clouds, const std::vector<cv::Mat> &transforms){
    if(in_clouds.size()!=transforms.size())
        throw runtime_error("Clouds vector and the transforms vector must have the same size.");
    std::vector<PointCloud> out_clouds(in_clouds.size());
    for(size_t i=0;i<out_clouds.size();i++)
        out_clouds[i]=in_clouds[i].transform(transforms[i]);
    return out_clouds;
}

std::vector<PointCloud> PointCloud::transformClouds(const std::vector<PointCloud>& in_clouds, const cv::Mat &transform){
    std::vector<PointCloud> out_clouds(in_clouds.size());
    for(size_t i=0;i<out_clouds.size();i++)
        out_clouds[i]=in_clouds[i].transform(transform);
    return out_clouds;
}
