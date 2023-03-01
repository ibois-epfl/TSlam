#include "rgbdreader.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include "filesystem.h"

using namespace std;

void print_binary(unsigned short input){
    unsigned short last_bit = 1;
    last_bit <<= 15;
    for(int i=0;i<16;i++){
        if((input&last_bit)>0)
            cout<<1;
        else
            cout<<0;
        input <<= 1;
    }
}

cv::Matx33d RGBDReader::getColorCamMat(){
    return rgb_cam_mat;
}

cv::Matx33d RGBDReader::getDepthCamMat(){
    return depth_cam_mat;
}

cv::Matx<double,5,1> RGBDReader::getColorDistCoeffs(){
    return rgb_dist_coeffs;
}

cv::Matx<double,5,1> RGBDReader::getDepthDistCoeffs(){
    return depth_dist_coeffs;
}

cv::Size RGBDReader::getColorImageSize(){
    return color_size;
}

cv::Size RGBDReader::getDepthImageSize(){
    return depth_size;
}

double RGBDReader::get_depth_in_meters(char16_t input){
    return double(input)/1000.0;
}

double RGBDReader::get_time_stamp(std::string file_name){
    return stod(file_name.substr(0,file_name.find_first_of('.')));
}

void RGBDReader::setRegistered(bool val){
    registered=val;
}

void RGBDReader::readCalib(string path){
    if(path.empty())
        return;

    cv::FileStorage fs(path,cv::FileStorage::READ);
    cv::Mat cmc,dcc,cmi,dci;
    fs["cam_mat_color"]>>cmc;
    rgb_cam_mat=cv::Matx33d(cmc);
    fs["dist_coeffs_color"]>>dcc;
    rgb_dist_coeffs=cv::Matx<double,5,1>(dcc);

    fs["cam_mat_ir"]>>cmi;
    depth_cam_mat=cv::Matx33d(cmi);
    fs["dist_coeffs_ir"]>>dci;
    depth_dist_coeffs=cv::Matx<double,5,1>(dci);

    cv::Size file_depth_size,file_color_size;
    fs["im_size_ir"]>>file_depth_size;
    fs["im_size_color"]>>file_color_size;

    if(color_size!=cv::Size(-1,-1)){
        double color_scale=color_size.width/double(file_color_size.width);
        rgb_cam_mat*=color_scale;
        rgb_cam_mat(2,2)=1;
    }

    if(depth_size!=cv::Size(-1,-1)){
        double depth_scale=depth_size.width/double(file_depth_size.width);
        depth_cam_mat*=depth_scale;
        depth_cam_mat(2,2)=1;
    }

    if(!fs["R"].empty() && !fs["t"].empty()){
        cv::Mat R,t;
        fs["R"]>>R;
        fs["t"]>>t;
        coordinate_transformer.set_R(R);
        coordinate_transformer.set_t(t);
    }
    else{//if the R and t do not exsist turn registration off
        registered=true;
        depth_cam_mat=rgb_cam_mat;
        depth_dist_coeffs=rgb_dist_coeffs;
        depth_size=color_size;
    }
}

void RGBDReader::find_correspondences(){
    set<string>::iterator closest_before=depth_file_names.begin();
    set<string>::iterator closest_after=depth_file_names.begin();

    for(string file_name: rgb_file_names){
        double rgb_ts=get_time_stamp(file_name);
        while(closest_after != depth_file_names.end()){
            if(get_time_stamp(*closest_after)<rgb_ts){
                closest_before = closest_after;
                closest_after ++;
            }
            else{
                break;
            }
        }

        if(closest_after == depth_file_names.end()){
            correspondences[file_name]=*closest_before;
            continue;
        }

        double closest_after_ts=get_time_stamp(*closest_after);
        double closest_before_ts=get_time_stamp(*closest_before);
        if( (closest_after_ts - rgb_ts) > (rgb_ts - closest_before_ts) )
            correspondences[file_name]=*closest_before;
        else
            correspondences[file_name]=*closest_after;
    }
    
    //fill correspondences_array for random access
    correspondences_array.clear();
    correspondences_array.reserve(correspondences.size());
    for(auto frame=correspondences.begin();frame!=correspondences.end();frame++)
        correspondences_array.push_back(frame);
    
}

bool RGBDReader::getCurrFrame(cv::Mat &depth, cv::Mat &color){
    curr_depth.copyTo(depth);
    curr_color.copyTo(color);
    return true;
}

void RGBDReader::getCurrColor(cv::Mat &color){
    curr_color.copyTo(color);
}

bool RGBDReader::readNextFrame(){
    if(curr_frame_num==-1){
        curr_frame=correspondences.begin();
    }
    else{
        curr_frame++;
    }
    curr_frame_num++;

    if(curr_frame==correspondences.end()){
        return false;
    }
    points_prepared=false;
    readCurrFrame();
    return true;
}

bool RGBDReader::getNextFrame(cv::Mat &depth, cv::Mat &color){
    if(!readNextFrame())
        return false;
    getCurrFrame(depth,color);
    return true;
}

bool RGBDReader::readFrameAt(unsigned long long frame_num){
    if(frame_num<correspondences.size()){
        curr_frame=correspondences_array[frame_num];
        curr_frame_num=frame_num;
    }
    else
        return false;

    readCurrFrame();
    return true;
}

bool RGBDReader::getFrameAt(unsigned long long frame_num, cv::Mat &depth, cv::Mat &color){
    if(!readFrameAt(frame_num))
        return false;
    getCurrFrame(depth,color);
    return true;
}

void RGBDReader::preparePoints(float max_dist, float min_dist){
    if(points_prepared)
        return;
    vector<cv::Point3f> point_3fs;
    vector<cv::Point2f> point_2fs;
    point_3fs.reserve(depth_size.area());
    
    cv::Mat depth;
    cv::Matx33d cam_mat_inv;
    if(registered && ref_cam_type==CamType::color){
        depth=undistort_color(curr_depth);
        cam_mat_inv=rgb_cam_mat.inv();
    }
    else{
        depth=undistort_depth(curr_depth);
        cam_mat_inv=depth_cam_mat.inv();
    }

    curr_points.clear();
    curr_projected_points.clear();
    
    for(int r=0;r<depth_size.height;r++){
        char16_t *depth_row=depth.ptr<char16_t>(r);
        for(int c=0;c<depth_size.width;c++){
            double z=get_depth_in_meters(depth_row[c]);
            if(z<min_dist || z>max_dist)
                continue;

            cv::Point3d point_3d(c,r,1);
            cv::Point3f point_3f=cam_mat_inv*point_3d*z;

            if(registered){
                curr_points.push_back(point_3f);
                curr_projected_points.emplace_back(c,r);
            }
            else{//if not registered we assume that the registration is from depth to rgb
                point_3f=coordinate_transformer(point_3f);//move to color cam coords
                point_3fs.push_back(point_3f);
            }
        }
    }

    if(!registered){
        //project the points
        vector<cv::Point2f> projected_points;
        if(point_3fs.size()>0)
            cv::projectPoints(point_3fs,cv::Mat::zeros(3,1,CV_64FC1),cv::Mat::zeros(3,1,CV_64FC1),rgb_cam_mat,rgb_dist_coeffs,projected_points);
        vector<long long int> pixel_points(color_size.area(),-1);
        for(size_t i=0;i<projected_points.size();i++){
            int x = projected_points[i].x + .5;//rounding to the closest pixel
            int y = projected_points[i].y + .5;
            if( x < color_size.width && x >=0 && y < color_size.height && y>=0){//in range
                //get the color
                size_t index=y*color_size.width+x;
                if(pixel_points[index]<0){//if it is the first point for that pixel add the point for that pixel
                    pixel_points[index]=i;
                }
                else if(point_3fs[i].z<point_3fs[pixel_points[index]].z){//if the z of the point is smaller than a previous point replace the previous point
                    pixel_points[index]=i;
                }
            }
        }
        for(size_t i=0;i<pixel_points.size();i++)
            if(pixel_points[i]>=0){
                curr_points.push_back(point_3fs[pixel_points[i]]);
                curr_projected_points.push_back(projected_points[pixel_points[i]]);
            }
    }
}

void RGBDReader::getXYZImage(cv::Mat &xyz, cv::Mat transform, float max_dist){
    if(curr_depth.empty())
        throw runtime_error("The current depth map is empty!");
    preparePoints(max_dist);
    xyz=cv::Mat(depth_size,CV_32FC3,cv::Scalar(NAN,NAN,NAN));
    //make fast access array
    vector<cv::Vec3f*> xyz_fast;
    for(int r=0;r<xyz.rows;r++)
        xyz_fast.push_back(xyz.ptr<cv::Vec3f>(r));
    //
    for(size_t i=0;i<curr_projected_points.size();i++){
        int x = curr_projected_points[i].x + .5;//rounding to the closest pixel
        int y = curr_projected_points[i].y + .5;
        cv::Point3f point_3D=curr_points[i];
        if(!transform.empty())
            point_3D=Transform3d(transform)(point_3D);
        xyz_fast[y][x]=point_3D;
    }
}

void RGBDReader::getCurrCloudAndNormals(std::vector<cv::Point3f> &points, std::vector<cv::Vec3b> &colors, std::vector<cv::Vec3f> &normals, Transform3d T, float max_dist){
    cv::Mat xyz;
    getXYZImage(xyz,cv::Mat(),max_dist);

    vector<cv::Vec3f*> xyz_fast;
    for(int r=0;r<xyz.rows;r++)
        xyz_fast.push_back(xyz.ptr<cv::Vec3f>(r));

    points.clear();
    colors.clear();
    normals.clear();

    cv::Matx33f R(T.get_R());

    for(int y=1;y<xyz.rows-1;y++){
        cv::Vec3b* fast_color_row=curr_color.ptr<cv::Vec3b>(y);
        for(int x=1;x<xyz.cols-1;x++){
            if(!isnan(xyz_fast[y][x][0])){//if a valid pixel to have normals
//                vector<cv::Point3f> valid_points;
//                valid_points.push_back(xyz_fast[y][x]);

//                for(int dx=-1;dx<=1;dx++)
//                    for(int dy=-1;dy<=1;dy++){
//                        cv::Vec3f point=xyz_fast[y+dy][x+dx];
//                        if(point!=zero_vec)
//                            valid_points.push_back(point);
//                    }
//                if(valid_points.size()<3)
//                    continue;

//                cv::Mat A(valid_points.size(),3,CV_32FC1);
//                cv::Mat b(valid_points.size(),1,CV_32FC1);
//                for(int r=0;r<A.rows;r++){
//                    float *fast_row=A.ptr<float>(r);
//                    fast_row[0]=valid_points[r].x;
//                    fast_row[1]=valid_points[r].y;
//                    fast_row[2]=1.0f;
//                    b.at<float>(r)=valid_points[r].z;
//                }
//                cv::Mat res=A.inv(cv::DECOMP_SVD)*b;
//                cv::Vec3f n(res);
//                n[3]=-1.0f;
//                n=cv::normalize(R*n);

//                normals.push_back(n);
//                cv::Point3f p=T(xyz_fast[y][x]);
//                points.push_back(p);
//                colors.push_back(fast_color_row[x]);

                cv::Vec3f neighbours[8];
                for(int i=0;i<8;i++){
                    int j=(i+2)%8;
                    int dx=0,dy=0;
                    if(i%4)
                        dx=(i/4)*2-1;
                    if(j%4)
                        dy=(j/4)*2-1;
                    neighbours[i]=xyz_fast[y+dy][x+dx];
                }

                cv::Vec3f point=xyz_fast[y][x],av_normal=cv::Vec3f(0,0,0);
                int num_normals=0;
                for(int i=0;i<8;i++)
                    if(!isnan(neighbours[i][0]) && !isnan(neighbours[(i+1)%8][0])){
                        av_normal+=cv::normalize((neighbours[i]-point).cross(neighbours[(i+1)%8]-point));
                        num_normals++;
                    }

                if(num_normals>0){
                    normals.push_back(R*cv::normalize(av_normal/num_normals));
                    points.push_back(T(xyz_fast[y][x]));
                    colors.push_back(fast_color_row[x]);
                }
            }
        }
    }
}

void RGBDReader::getCurrCloud(std::vector<cv::Point3f> &points, std::vector<cv::Vec3b> &colors, Transform3d T, float max_dist){

    if(curr_depth.empty())
        throw runtime_error("The current depth image is empty!");
    preparePoints(max_dist);

    cv::Mat rgb;
    cv::cvtColor(curr_color,rgb,CV_BGR2RGB);
    for(cv::Point3f point:curr_points)
        points.push_back(T(point));

    //make fast access arrays
    vector<cv::Vec3b*> rgb_fast;
    for(int r=0;r<rgb.rows;r++)
        rgb_fast.push_back(rgb.ptr<cv::Vec3b>(r));

    for(size_t i=0;i<curr_projected_points.size();i++){
        int x = curr_projected_points[i].x + .5;//rounding to the closest pixel
        int y = curr_projected_points[i].y + .5;
        colors.push_back(rgb_fast[y][x]);
    }

}



