#ifndef POINTCLOUD_H
#define POINTCLOUD_H
#include<vector>
#include<opencv2/core.hpp>
class PointCloud{
public:
    std::vector<cv::Point3f> points;
    std::vector<cv::Vec3b> colors;
    std::vector<cv::Vec3f> normals;
    PointCloud();
    PointCloud(const std::vector<cv::Point3f> &in_points, const std::vector<cv::Vec3b> &in_colors);
    void clear();
    bool empty();
    size_t size() const;
    void reverseNormals();
    PointCloud transform(const cv::Mat &T) const;
    static std::vector<PointCloud> transformClouds(const std::vector<PointCloud>& in_clouds, const cv::Mat &transform);
    static std::vector<PointCloud> transformClouds(const std::vector<PointCloud>& in_clouds, const std::vector<cv::Mat> &transforms);
    static PointCloud combineClouds(const std::vector<PointCloud> &in_clouds);
    static void get_Rt(const cv::Mat &T, cv::Matx33f &R, cv::Point3f &t);
    PointCloud operator + (const PointCloud& in_pc) const;
};
#endif // POINTCLOUD_H
