#ifndef POINTGRID_H
#define POINTGRID_H

#include "pointcloud.h"
#include <opencv2/core.hpp>

class PointGrid
{
    cv::Mat grid,color,mask,normals,T;

    public:

    PointGrid();
    PointGrid(const cv::Mat grid, const cv::Mat T=cv::Mat(), const cv::Mat color=cv::Mat(), const cv::Mat mask=cv::Mat());

    cv::Mat getGridMat();
    cv::Mat getNormalsMat();
    cv::Mat getPlaneDists(std::pair<cv::Vec3f,float>);
    void setNormalsMat(const cv::Mat &input);
    PointGrid getSmoothGrid(int radius, bool blur_color=false);
    void calcNormals();
    PointCloud getPointCloud();
};

#endif // POINTGRID_H
