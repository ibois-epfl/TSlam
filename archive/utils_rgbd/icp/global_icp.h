#ifndef GLOBAL_ICP_H
#define GLOBAL_ICP_H

#include "icp.h"

class Global_ICP
{

    std::vector<picoflann::KdTreeIndex<3,picoflann_point3f_adaptor>> trees;
    std::vector<PointCloud> clouds;
    std::vector<cv::Point3d> cloud_means;
    std::vector<std::vector<cv::Mat>> transforms;
    size_t num_clouds=0;
    void get_could_means();

public:
    Global_ICP(const std::vector<PointCloud> &clouds, float subsample_radius=0.01);
    Global_ICP(const std::vector<cv::Mat> &in_depthmaps, float subsample_radius=0.01);
    void initialize(float subsample_radius);
    void execute(std::pair<double,double> radius_range=std::pair<double,double>(0.01,0.05), int num_iterations=10, bool point_to_plane=true, bool neighbours_only=false, bool all_clouds=false);
    std::vector<std::vector<cv::Point3f>> getClouds(const std::vector<std::vector<cv::Point3f>> &in_clouds, int num_iterations=-1);
    std::vector<std::vector<cv::Mat>> getTransforms();
    std::vector<cv::Mat> getFinalTransforms(int num_iterations=-1);

    int importance(double distance,double max_distance,int iteration,int num_iterations);
    template<typename T>
    void subsampleDepthmap2Pointcloud(cv::Mat &depthmap,std::vector<cv::Point3f> &cloud, float subsample_radius);
};

#endif // GLOBAL_ICP_H
