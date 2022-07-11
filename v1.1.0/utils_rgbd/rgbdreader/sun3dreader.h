#ifndef SUN3DREADER_H
#define SUN3DREADER_H

#include "rgbdreader.h"
#include <set>
#include <map>

class SUN3DReader: public RGBDReader
{    
    double get_depth_in_meters(char16_t);
public:
    bool readCurrFrame();
    cv::Point3f move_depth_to_color_coords(cv::Point3f);
    cv::Mat undistort_color(cv::Mat);
    cv::Mat undistort_depth(cv::Mat);
    double get_time_stamp(std::string input);
    SUN3DReader(std::string folder_path);
};

#endif // SUN3DREADER_H
