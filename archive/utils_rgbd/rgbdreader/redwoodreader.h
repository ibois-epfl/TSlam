#ifndef REDWOODREADER_H
#define REDWOODREADER_H

#include "rgbdreader.h"
class RedwoodReader: public RGBDReader
{
    cv::Point3f move_depth_to_color_coords(cv::Point3f);
    cv::Mat undistort_depth(cv::Mat);
    cv::Mat undistort_color(cv::Mat);
    double get_time_stamp(std::string file_name);
public:
    bool readCurrFrame();
    RedwoodReader(std::string path);
};

#endif // REDWOODREADER_H
