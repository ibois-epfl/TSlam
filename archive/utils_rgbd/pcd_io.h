#ifndef PCD_IO_H
#define PCD_IO_H
#include<opencv2/core.hpp>
#include "pointcloud.h"
class PCD_IO
{
public:
    static void read_pcd(std::string path, std::vector<cv::Point3f> &points, std::vector<cv::Vec3b> *colors=NULL, std::vector<cv::Vec3f> *normals=NULL);
    static PointCloud read_pcd(std::string path);
    static void write_ply(std::string path, const PointCloud &pc);
    static void write_pcd(std::string path, const std::vector<cv::Point3f> &points, const std::vector<cv::Vec3b> &colors=std::vector<cv::Vec3b>(), const std::vector<cv::Vec3f> &normals=std::vector<cv::Vec3f>());
    static void write_pcd(std::string path, const PointCloud &pc);
    static void write_pcd(std::string path, cv::Mat &xyz, cv::Mat &color_image);
    PCD_IO();
};

#endif // PCD_IO_H
