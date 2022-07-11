#ifndef NYUREADER_H
#define NYUREADER_H
#include "rgbdreader.h"

class NYUReader: public RGBDReader
{
    cv::Point3f move_depth_to_color_coords(cv::Point3f);
    std::string dataset_version;
    double get_time_stamp(std::string);
    double get_depth_in_meters(char16_t);
    cv::Mat undistort_depth(cv::Mat);
    cv::Mat undistort_color(cv::Mat);
public:
    NYUReader(std::string path, std::string d_version="v2");
    bool readCurrFrame();
};

#endif // NYUREADER_H
