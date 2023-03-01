#ifndef ICP_H
#define ICP_H

#include <random>
#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include "basictypes/picoflann.h"
#include "../pointcloud.h"


struct picoflann_point3f_adaptor{
    inline float operator()(const cv::Point3f &input, int &dim) const{
        return cv::Vec3f(input)[dim];
    }
    inline float operator()(const cv::Point3f &input, const uint16_t &dim) const{
        return cv::Vec3f(input)[dim];
    }
};

class ICP
{
    PointCloud source;
    PointCloud dest;
public:
    ICP(const PointCloud &src, const PointCloud &dst, float ss_radius=0.0);
    ICP(const std::vector<cv::Point3f> &src, const std::vector<cv::Point3f> &dst, float ss_radius=0.0, const std::vector<cv::Vec3f> &src_normals=std::vector<cv::Vec3f>(), const std::vector<cv::Vec3f> &dst_normals=std::vector<cv::Vec3f>());
    void init(const PointCloud &src, const PointCloud &dst, float ss_radius=0);

    static pcl::PointXYZINormal getPCLPoint(const cv::Point3f &point, const cv::Vec3f &normal);
    static void get_Rt(const cv::Mat T, cv::Matx33d &R, cv::Point3d &t);
    static void get_Rt(const cv::Mat T, cv::Matx33f &R, cv::Point3f &t);
    static void transformPointcloud(std::vector<cv::Point3f> &cloud, const cv::Mat T);
    static PointCloud subsamplePointCloud(const PointCloud &in_cloud, float radius, unsigned int seed);
    static std::vector<cv::Point3f> subsamplePointCloud(const std::vector<cv::Point3f> &in_cloud, float radius, unsigned int seed);
    cv::Mat exec(std::pair<double,double> radius_range, int num_iterations, bool point_to_plane=false);
};

#endif // ICP_H
