#ifndef HEIGHTMAP_H
#define HEIGHTMAP_H
#include <opencv2/core.hpp>
#include "pointcloud.h"
#include "pointgrid.h"

class HeightMap
{
public:
    HeightMap();
    PointCloud getPointcloud();
    HeightMap getBlurredMap(cv::Size filter_size, bool blur_color=false);
    HeightMap getDiffFrom(const HeightMap &ref, float max_abs_diff=0.1);
    static std::vector<HeightMap> fromPointClouds(const std::vector<PointCloud> &pc, const cv::Rect2f &in_ranges, float in_grid_step, float in_surface_thickness);
    static HeightMap merge(const std::vector<HeightMap> &in_hms);
    void fromPointCloud(const PointCloud &pc, const cv::Rect2f &in_ranges, float in_grid_step, float in_surface_thickness);
    void init_mats(int rows, int cols);
    void readFromFile(std::string path);
    void writeToFile(std::string path);
    bool get3DCoords(cv::Point, cv::Point3f&);
    cv::Mat getRGB() const;
    cv::Point3f get3DCoords(cv::Point2f);
    cv::Mat getHeightImage() const;

    PointGrid getPointGrid() const;

    cv::Rect2f ranges;
    float grid_step, surface_thickness;
    cv::Mat height_map, map_color, mask;
};

#endif // HEIGHTMAP_H
