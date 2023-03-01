#include "nyureader.h"
#include "filesystem.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
// #include <iostream>
#include <iomanip>
using namespace std;
NYUReader::NYUReader(string path, string version)
{
    dir_path=path;
    if(version=="v1")
        throw runtime_error("NYUReader for the NYU dataset version 1 is still not implemented.");

    rgb_dist_coeffs=cv::Matx<double,5,1>(2.0796615318809061e-01,-5.8613825163911781e-01,7.2231363135888329e-04,1.0479627195765181e-03,4.9856986684705107e-01);
    depth_dist_coeffs=cv::Matx<double,5,1>(-9.9897236553084481e-02,3.9065324602765344e-01,1.9290592870229277e-03,-1.9422022475975055e-03,-5.1031725053400578e-01);

    rgb_cam_mat=cv::Matx33d(5.1885790117450188e+02, 0., 3.2558244941119034e+02,
                            0., 5.1946961112127485e+02, 2.5373616633400465e+02,
                            0.,                     0.,                    1.0);

    depth_cam_mat=cv::Matx33d(5.8262448167737955e+02,   0., 3.1304475870804731e+02,
                              0.,   5.8269103270988637e+02, 2.3844389626620386e+02,
                              0.,                     0.,                      1.0);
    cv::Mat empty_mat;
    cv::initUndistortRectifyMap(rgb_cam_mat,rgb_dist_coeffs,cv::Mat(),rgb_cam_mat,cv::Size(640,480),CV_16SC2,rgb_undistort_map,empty_mat);
    cv::initUndistortRectifyMap(depth_cam_mat,depth_dist_coeffs,cv::Mat(),depth_cam_mat,cv::Size(640,480),CV_16SC2,depth_undistort_map,empty_mat);

    coordinate_transformer=cv::Matx44d(	9.9997798940829263e-01,	5.0518419386157446e-03,	4.3011152014118693e-03,2.5031875059141302e-02,
					-5.0359919480810989e-03,9.9998051861143999e-01,-3.6879781309514218e-03,-2.9342312935846411e-04,
				       -4.3196624923060242e-03,	3.6662365748484798e-03, 9.9998394948385538e-01,6.6238747008330102e-04,
					0 			,0 			,0			,1
    );
    
    dataset_version=version;
    set<string> files_list=filesystem::get_files_list(path);
    for(string file_name:files_list)
        if(file_name[0]=='d')
            depth_file_names.insert(file_name);
        else if(file_name[0]=='r')
            rgb_file_names.insert(file_name);

    depth_size=color_size=cv::Size(640,480);

    find_correspondences();
}

double NYUReader::get_time_stamp(string file_name){
    size_t b=file_name.find_first_of('-');
    size_t e=file_name.find_last_of('-');
    return stod(file_name.substr(b+1,e-b-1));
}

double NYUReader::get_depth_in_meters(char16_t input){
    double d=351.3/(1092.5-input);
    d=std::max(d,0.0);
    d=std::min(d,10.0);
    return d;
}

bool NYUReader::readCurrFrame(){
    
    curr_color=cv::imread(dir_path+"/"+curr_frame->first);
    curr_depth=cv::imread(dir_path+"/"+curr_frame->second,cv::IMREAD_UNCHANGED);

    for(int r=0;r<curr_depth.rows;r++){
        char16_t *row=curr_depth.ptr<char16_t>(r);
        for(int c=0;c<curr_color.cols;c++){
            char byte, *bytes=(char*)&row[c];
            byte = bytes[0];
            bytes[0] = bytes[1];
            bytes[1] = byte;
        }
    }
    return true;
}

cv::Mat NYUReader::undistort_depth(cv::Mat depth){
    cv::Mat undistorted_depth;
    cv::remap(depth,undistorted_depth,depth_undistort_map,cv::Mat(),cv::INTER_NEAREST);
    return undistorted_depth;
}

cv::Mat NYUReader::undistort_color(cv::Mat color){
    cv::Mat undistorted_color;
    cv::remap(color,undistorted_color,rgb_undistort_map,cv::Mat(),cv::INTER_NEAREST);
    return undistorted_color;
}

cv::Point3f NYUReader::move_depth_to_color_coords(cv::Point3f input){
    return coordinate_transformer(input);
}
