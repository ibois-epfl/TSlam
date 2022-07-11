#include "redwoodreader.h"
#include "filesystem.h"
#include <opencv2/highgui.hpp>

using namespace std;

RedwoodReader::RedwoodReader(std::string path)
{
  dir_path = path;
  
  rgb_cam_mat=depth_cam_mat=cv::Matx33d(525,	0,	319.5,
					0,	525,	239.5,
					0,	0,	1);
  
  depth_size=color_size=cv::Size(640,480);
  rgb_dist_coeffs=depth_dist_coeffs=cv::Matx<double,5,1>::zeros();
  
  set<string> dirs_list=filesystem::get_dirs_list(path);
  if(dirs_list.count("depth")==0 || dirs_list.count("image")==0)
    throw runtime_error("There is no depth and/or image folder in the specified Redwood dataset folder.");
  
  set<string> depth_files=filesystem::get_files_list(path+"/depth/");
  set<string> rgb_files=filesystem::get_files_list(path+"/image/");
  depth_file_names=depth_files;
  rgb_file_names=rgb_files;
  
  
  find_correspondences();
}

bool RedwoodReader::readCurrFrame(){
    
    curr_color=cv::imread(dir_path+"/image/"+curr_frame->first);
    curr_depth=cv::imread(dir_path+"/depth/"+curr_frame->second,cv::IMREAD_UNCHANGED);

    return true;
}

double RedwoodReader::get_time_stamp(std::string file_name){
    return stod(file_name.substr(0,file_name.find('.')));
}

cv::Point3f RedwoodReader::move_depth_to_color_coords(cv::Point3f input){
    return input;
}
    
cv::Mat RedwoodReader::undistort_depth(cv::Mat input){
    return input;
}
    
cv::Mat RedwoodReader::undistort_color(cv::Mat input){
    return input;
}
