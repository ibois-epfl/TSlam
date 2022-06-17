#include "sun3dreader.h"
#include <fstream>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <filesystem.h>

using namespace std;

SUN3DReader::SUN3DReader(string path)
{
    dir_path=path;
    set<string> dirs = filesystem::get_dirs_list(path);

    if(dirs.count("depth")==0 || dirs.count("image")==0)
        throw runtime_error("The \"depth\" and/or the \"image\" folder do not exist in the data folder path provided: \n"+path);

    set<string> files = filesystem::get_files_list(path);
    if(files.count("intrinsics.txt")==0)
        throw runtime_error("No intrinsics.txt file was found in the folder: \n"+path);

    //read the camera matrix
    ifstream intrinsics(path+"/intrinsics.txt");
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            intrinsics>>rgb_cam_mat(i,j);
    
    rgb_dist_coeffs=depth_dist_coeffs=cv::Matx<double,5,1>::zeros();

    rgb_file_names = filesystem::get_files_list(path+"/image");
    depth_file_names = filesystem::get_files_list(path+"/depth");

    depth_size=color_size=cv::Size(640,480);

    find_correspondences();
}

double SUN3DReader::get_depth_in_meters(char16_t input){
    return double(input)/1000.0;
}

double SUN3DReader::get_time_stamp(string input){
    int dash_pos=input.find('-');
    int dot_pos=input.find('.');
    return std::stod(input.substr(dash_pos+1,dot_pos-dash_pos-1));
}

bool SUN3DReader::readCurrFrame(){

    curr_color=cv::imread(dir_path+"/image/"+curr_frame->first);
    curr_depth=cv::imread(dir_path+"/depth/"+curr_frame->second,cv::IMREAD_UNCHANGED);

    for(int r=0;r<curr_depth.rows;r++){
        char16_t *row=curr_depth.ptr<char16_t>(r);
        for(int c=0;c<curr_depth.cols;c++){
            char16_t val=row[c];
            row[c]=((val<<13)|(val>>3));
        }
    }

    return true;
}

cv::Point3f SUN3DReader::move_depth_to_color_coords(cv::Point3f depth){
    return depth;
}

cv::Mat SUN3DReader::undistort_depth(cv::Mat input){
    return input;
}

cv::Mat SUN3DReader::undistort_color(cv::Mat input){
    return input;
}
