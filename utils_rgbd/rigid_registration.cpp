#include <iostream>
#include "map.h"
#include <cmath>
#include "rgbdreaderfactory.h"
#include "sequenceoptimizer.h"
#include "sparselevmarq.h"
#include <limits>
#include <cstdio>
#include <opencv2/highgui.hpp>
#include "basictypes/misc.h"
#include "global_icp.h"
#include "pointcloud.h"
#include "heightmap.h"
#include "pointgrid.h"
#include "pcd_io.h"
#include <aruco/aruco.h>

using namespace std;

cv::Mat get_axis_angle(cv::Vec3f cam_dir) {
    cv::Vec3d z(0,0,1);
    cv::Vec3d dir(cam_dir);
    cv::Vec3d axis;
    if(dir==z)
        axis=z;
    else
        axis=z.cross(dir);
    double theta = std::acos(z.dot(dir));
//    cout<<"cam_dir"<<dir<<endl;
//    cout<<"axis"<<axis<<endl;
    axis/=cv::norm(axis);
    axis*=theta;
    return cv::Mat(axis);
}

cv::Mat getMatrix(double qx,double qy, double qz,double qw,double tx,double ty ,double tz) { //from rafa


    double qx2 = qx*qx;
    double qy2 = qy*qy;
    double qz2 = qz*qz;


    cv::Mat m=cv::Mat::eye(4,4,CV_64F);

    m.at<double>(0,0)=1 - 2*qy2 - 2*qz2;
    m.at<double>(0,1)=2*qx*qy - 2*qz*qw;
    m.at<double>(0,2)=2*qx*qz + 2*qy*qw;
    m.at<double>(0,3)=tx;

    m.at<double>(1,0)=2*qx*qy + 2*qz*qw;
    m.at<double>(1,1)=1 - 2*qx2 - 2*qz2;
    m.at<double>(1,2)=2*qy*qz - 2*qx*qw;
    m.at<double>(1,3)=ty;

    m.at<double>(2,0)=2*qx*qz - 2*qy*qw;
    m.at<double>(2,1)=2*qy*qz + 2*qx*qw;
    m.at<double>(2,2)=1 - 2*qx2 - 2*qy2;
    m.at<double>(2,3)=tz;
    return m;
}

void read_orbslam2_keyframes(string file_path,map<unsigned long, unsigned long> &frame2keyframe_index, map<unsigned long, cv::Mat> &orbslam2_pose_converters, unsigned long &last_frame_index) {
    frame2keyframe_index.clear();
    orbslam2_pose_converters.clear();
    std::ifstream input_file(file_path);
    double frame_num,x,y,z,qx,qy,qz,qw;
    int key_frame_index=0;
    while(input_file>>frame_num>>x>>y>>z>>qx>>qy>>qz>>qw) {
        x/=1000.0;
        y/=1000.0;
        z/=1000.0;
        frame2keyframe_index[frame_num]=key_frame_index++;
        orbslam2_pose_converters[frame_num]=getMatrix(qx,qy,qz,qw,x,y,z);
        last_frame_index=frame_num;
    }
}

void draw_3d_line_points(cv::Point3f start, const cv::Point3f end, int steps, cv::Vec3b start_color, cv::Vec3b end_color, PointCloud& pc) {
    cv::Point3d step=(end-start)/steps;
    cv::Vec3d color_step=(cv::Vec3d(end_color)-cv::Vec3d(start_color))/double(steps);
    cv::Point3d point=start;
    for(int i=0; i<steps; i++,point+=step) {
        pc.points.push_back(point);
        pc.colors.push_back(start_color+cv::Vec3b(i*color_step));
        pc.normals.emplace_back(0,0,0);
    }
}

void visualize_marker_3d(cv::Mat corners, PointCloud& pc){
    for(int i=0;i<4;i++){
        draw_3d_line_points(cv::Point3d(corners.col(i)),cv::Point3d(corners.col((i+1)%4)),100,cv::Vec3b(255,0,0),cv::Vec3b(255,0,0),pc);
    }
}

void visualize_marker_3d(const vector<cv::Vec3d> &corners, PointCloud& pc){
    for(int i=0;i<4;i++){
        draw_3d_line_points(cv::Point3d(corners[i]),cv::Point3d(corners[(i+1)%4]),100,cv::Vec3b(255,0,0),cv::Vec3b(255,0,0),pc);
    }
}

void visualize_marker_3d(const vector<cv::Point3f> &corners, PointCloud& pc){
    for(int i=0;i<4;i++){
        draw_3d_line_points(corners[i],corners[(i+1)%4],100,cv::Vec3b(255,0,0),cv::Vec3b(255,0,0),pc);
    }
}

vector<map<unsigned int,cv::Point3f>> mappoint_keyframe_keypoint_coords;
vector<cv::Point3f> mappoint_coords;
map<unsigned int, cv::Mat> keyframe_transforms;
map<unsigned int, unsigned int> keyframe2frame_index;
map<unsigned int, unsigned int> frame2keyframe_index;
map<unsigned int, unsigned int> keyframe_io_index;

void getTransformsPointCloud(shared_ptr<RGBDReader> rgbd_reader, PointCloud &pc, map<unsigned int,cv::Mat>& transforms){
    pc.points.clear();
    pc.colors.clear();
    for(auto &frame2keyframe: frame2keyframe_index){
        unsigned int frame_index=frame2keyframe.first;
        unsigned int keyframe_index=frame2keyframe.second;

        cout<<frame_index<<endl;
//        cout<<transforms[keyframe_index]<<endl;
        rgbd_reader->readFrameAt(frame_index);
        rgbd_reader->getCurrCloud(pc.points,pc.colors,transforms[keyframe_index]);
    }
}

void keyframe_transforms_to_io_vec(reslam::SparseLevMarq<double>::eVector &io_vec){
    io_vec.resize(keyframe_transforms.size()*6);
    unsigned int curr_index=0;
    for(std::pair<const unsigned int,cv::Mat> &keyframe_transform:keyframe_transforms){
        cv::Mat transform=keyframe_transform.second;
        cv::Mat r;
        cv::Rodrigues(transform(cv::Range(0,3),cv::Range(0,3)),r);
        for(int i=0;i<3;i++){
            io_vec(curr_index+i)=r.at<float>(i);
            io_vec(curr_index+i+3)=transform.at<float>(i,3);
        }
        curr_index+=6;
    }
}

void io_vec_to_keyframe_transforms(reslam::SparseLevMarq<double>::eVector &io_vec){
    unsigned int curr_index=0;
    for(std::pair<const unsigned int,cv::Mat> &keyframe_transform:keyframe_transforms){
        cv::Mat transform=cv::Mat::eye(4,4,CV_32FC1);
        cv::Mat r(3,1,CV_32FC1);
        for(int i=0;i<3;i++){
            r.at<float>(i)=io_vec(curr_index+i);
            transform.at<float>(i,3)=io_vec(curr_index+i+3);
        }
        cv::Rodrigues(r,transform(cv::Range(0,3),cv::Range(0,3)));
        keyframe_transform.second=transform;
        curr_index+=6;
    }
}

void error_function(const typename reslam::SparseLevMarq<double>::eVector &input, typename reslam::SparseLevMarq<double>::eVector &error){

    vector<vector<vector<double>>> mappoint_diffs(mappoint_keyframe_keypoint_coords.size());
    unsigned int mappoint_index=0;
    unsigned int total_errors=0;
    for(auto &keyframe_keypoint_coords:mappoint_keyframe_keypoint_coords){//loop over each mappoint
        vector<cv::Point3f> transformed_points;
        cv::Point3f sum(0,0,0);
        mappoint_diffs[mappoint_index].resize(keyframe_keypoint_coords.size());
        unsigned int keyframe_index=0;
        for(auto &keypoint_coords:keyframe_keypoint_coords){//loop over each corresponding keyframe
            unsigned int io_index=keyframe_io_index[keypoint_coords.first];
            cv::Vec3f coords=keypoint_coords.second;
            cv::Vec3f r,t;
            cv::Matx33f R;
            for(int i=0;i<3;i++){
                r(i)=input(io_index+i);
                t(i)=input(io_index+3+i);
            }
            cv::Rodrigues(r,R);
            cv::Point3f transformed_coords=R*coords+t;
            sum+=transformed_coords;
            transformed_points.push_back(transformed_coords);
        }
        cv::Point3f average=sum/double(transformed_points.size());
        for(size_t i=0;i<transformed_points.size();i++){
            mappoint_diffs[mappoint_index][keyframe_index].push_back(transformed_points[i].x-average.x);
            mappoint_diffs[mappoint_index][keyframe_index].push_back(transformed_points[i].y-average.y);
            mappoint_diffs[mappoint_index][keyframe_index].push_back(transformed_points[i].z-average.z);
            total_errors+=3;
        }
//        double sum_x_diff, sum_y_diff, sum_z_diff;
//        sum_x_diff=sum_y_diff=sum_z_diff=0;
//        for(size_t i=0;i<transformed_points.size();i++){
//            sum_x_diff+=std::abs(transformed_points[i].x-average.x);
//            sum_y_diff+=std::abs(transformed_points[i].y-average.y);
//            sum_z_diff+=std::abs(transformed_points[i].z-average.z);
//        }
//        error(error_index)=sum_x_diff;
//        error(error_index+1)=sum_y_diff;
//        error(error_index+2)=sum_z_diff;
//        error_index+=3;
        mappoint_index++;
    }
    unsigned int error_index=0;
    error.resize(total_errors);
    for(size_t i=0;i<mappoint_diffs.size();i++)
        for(size_t j=0;j<mappoint_diffs[i].size();j++)
            for(size_t k=0;k<mappoint_diffs[i][j].size();k++)
                error(error_index++)=mappoint_diffs[i][j][k];
}

void draw_keypoint_lines(PointCloud &pc){
    for(auto &keyframe_keypoint_coords:mappoint_keyframe_keypoint_coords){
        cv::Point3f average(0,0,0);
        vector<cv::Point3f> global_coords;
        for(auto &keypoint_coords:keyframe_keypoint_coords){
            unsigned int keyframe_index=keypoint_coords.first;
            global_coords.push_back(Transform3d(keyframe_transforms[keyframe_index])(keypoint_coords.second));
            average+=global_coords.back();
        }
        average/=double(global_coords.size());
        for(auto &global_coord:global_coords)
            draw_3d_line_points(global_coord,average,100,cv::Vec3b(0,0,0),cv::Vec3b(255,0,0),pc);
    }
}

void get_keyframe_transforms(reslam::Map &m){
    for(auto &keyframe:m.keyframes){
        keyframe.pose_f2g.inv().copyTo(keyframe_transforms[keyframe.idx]);
    }
}

void get_keyframe_transforms_and_3D_keypoint_coords(reslam::Map &m){//uses map mpoint to filter keyframes (usefult when using keypoints to register the clouds)
    for(reslam::MapPoint& mappoint:m.map_points) {
        map<unsigned int,cv::Point3f> keyframe_keypoint_coords;

        if(!mappoint.isBad()){
            const auto &mappoint_keyframes=mappoint.getObservingFrames();
            map<unsigned int,cv::Point3f> keypoint_global_positions;
            for(auto &mappoint_keyframe: mappoint_keyframes){//go through the observing keypoints
                unsigned int keyframe_index=mappoint_keyframe.first;
                unsigned int keypoint_index=mappoint_keyframe.second;

                reslam::Frame &keyframe=m.keyframes[keyframe_index];
                cv::KeyPoint keypoint=keyframe.und_kpts[keypoint_index];
                //only use the keypoints in the first octave
                if(keyframe.getDepth(keypoint_index)<=2.0 && keypoint.octave==0 && !std::isnan(keypoint.pt.x) && !std::isnan(keypoint.pt.y) && !keyframe.isBad()){
                    if(keyframe.getDepth(keypoint_index)>0){//check if the keypoint has a valid depth
                        keyframe_keypoint_coords[keyframe_index]=keyframe.get3dStereoPoint(keypoint_index);
                        if(keyframe_transforms.count(keyframe_index)==0){//save the transformation of the
                            keyframe.pose_f2g.inv().copyTo(keyframe_transforms[keyframe_index]);
                        }
                        keypoint_global_positions[keyframe_index]=Transform3d(keyframe_transforms[keyframe_index])(keyframe_keypoint_coords[keyframe_index]);
                    }
                }
            }

            if(keypoint_global_positions.size()>0){
                //get the average
                cv::Point3f average(0,0,0);
                for(auto &global_position : keypoint_global_positions)
                    average+=global_position.second;
                average/=double(keypoint_global_positions.size());

                //get the diffs
                vector<pair<unsigned int,double>> keyframe_diffs;
                for(auto &global_position : keypoint_global_positions)
                    keyframe_diffs.push_back(make_pair(global_position.first,cv::norm(global_position.second-average)));

                //get the mdedian

                std::sort(keyframe_diffs.begin(),keyframe_diffs.end(),[](pair<unsigned int,double> l,pair<unsigned int,double> r){
                   return l.second>r.second;
                });

                unsigned int median_keyframe_index1=keyframe_diffs[keyframe_diffs.size()/2].first;

                cv::Point3f median;


                if(keyframe_diffs.size()%2==1){
                    median=keypoint_global_positions[median_keyframe_index1];
                }
                else{
                    unsigned int median_keyframe_index2=keyframe_diffs[keyframe_diffs.size()/2+1].first;
                    median=(keypoint_global_positions[median_keyframe_index1]+keypoint_global_positions[median_keyframe_index2])/2.0;
                }

                //find the outliers and remove them
                for(auto &global_position : keypoint_global_positions){
                    if(cv::norm(median-global_position.second)>0.1){//the distance to median is more that 10 centimeters
                        unsigned int keyframe_index=global_position.first;
                        keyframe_keypoint_coords.erase(keyframe_index);
                    }
                }

            }

            if(keyframe_keypoint_coords.size()>0){
                mappoint_keyframe_keypoint_coords.push_back(keyframe_keypoint_coords);
                mappoint_coords.push_back(mappoint.getCoordinates());
            }
        }
    }
}

//void register_keyframe_clouds(vector<depthmaps::EmbeddedRegistration> &ers){//deformable registration
//    map<unsigned int, vector<cv::Vec3f>> keyframe_mappoint_coords;
//    map<unsigned int, vector<cv::Vec3f>> keyframe_keypoint_global_coords;

//    ers.resize(keyframe2frame_index.size());

//    for(size_t i=0;i<mappoint_keyframe_keypoint_coords.size();i++){
//        auto &keyframe_keypoint_coords=mappoint_keyframe_keypoint_coords[i];
//        for(pair<unsigned int,cv::Point3f> p:keyframe_keypoint_coords){//move the keypoints from the keframe coordinates to the global coordinates
//            unsigned int keyframe_index=p.first;
//            cv::Point3f coords=Transform3d(keyframe_transforms[keyframe_index])(p.second);
//            keyframe_keypoint_global_coords[keyframe_index].push_back(coords);
//            keyframe_mappoint_coords[keyframe_index].push_back(mappoint_coords[i]);
//        }
//    }

//    size_t i=0;
//    for(std::pair<const unsigned int,unsigned int> &p:keyframe2frame_index){
//        unsigned int keyframe_index=p.first;
//        depthmaps::EmbeddedRegistration::Graph graph;
//        graph.create(keyframe_mappoint_coords[keyframe_index],0,4);//create a graph using the mappoint coordinates
//        ers[i].setParams(keyframe_keypoint_global_coords[keyframe_index],keyframe_mappoint_coords[keyframe_index],graph);//get the deformable registration for that keyframe
//        ers[i].run();
//        i++;
//    }
//}

//void apply_keyframe_deformations(vector<depthmaps::EmbeddedRegistration> &ers,const vector<vector<cv::Point3f>> &keyframe_clouds, vector<vector<cv::Point3f>> &keyframe_deformed_clouds){
//    keyframe_deformed_clouds.resize(keyframe_clouds.size());
//    for(size_t i=0;i<ers.size();i++){
//        vector<cv::Vec3f> keyframe_cloud(keyframe_clouds.size()),keyframe_deformed_cloud(keyframe_clouds.size());
//        for(size_t j=0;j<keyframe_clouds.size();j++)
//            keyframe_cloud[j]=keyframe_clouds[i][j];

//        ers[i].apply(keyframe_cloud,keyframe_deformed_cloud);

//        for(size_t j=0;j<keyframe_clouds.size();j++)
//            keyframe_deformed_clouds[i][j]=keyframe_deformed_cloud[j];
//    }
//}

//void XYZImage_2_depthmap(const cv::Mat imgXYZ, const cv::Mat image,depthmaps::DepthMap &dm){
//    if(dm.size!=imgXYZ.size)
//        dm.create(imgXYZ.rows,imgXYZ.cols,true);
//    for(int i=0;i<imgXYZ.rows;i++){
//        depthmaps::DepthPixel *dm_row=dm.ptr<depthmaps::DepthPixel>(i);
//        const cv::Vec3f *ixyz_row=imgXYZ.ptr<cv::Vec3f>(i);
//        const cv::Vec3b *image_row=image.ptr<cv::Vec3b>(i);
//        for(int j=0;j<imgXYZ.cols;j++)
//            if(ixyz_row[j]!=cv::Vec3f(0,0,0)){
//                dm_row[j]=ixyz_row[j];
//                cv::Vec3b color=image_row[j];
//                dm_row[j].setRGB(color[2],color[1],color[0]);
//            }
//    }
//}

void read_keyframe_pointclouds(shared_ptr<RGBDReader> rgbd_reader, vector<PointGrid> &grids, double max_dist=2){
    for(std::pair<const unsigned int,unsigned int> &p:frame2keyframe_index){
        unsigned int frame_index=p.first;
        unsigned int keyframe_index=p.second;

        rgbd_reader->readFrameAt(frame_index);

        cv::Mat xyz,rgb;
        rgbd_reader->getCurrColor(rgb);
        rgbd_reader->getXYZImage(xyz,cv::Mat(),max_dist);
        cv::cvtColor(rgb,rgb,cv::COLOR_BGR2RGB);
        grids.emplace_back(xyz,keyframe_transforms[keyframe_index],rgb);

    }
}

//void transform_pointcloud(depthmaps::PointCloud &cloud,cv::Mat T){
//    T.convertTo(T,CV_32FC1);
//    for(int i=0;i<cloud.size();i++){
//        cv::Mat v(4,1,CV_32FC1);

//        cv::Vec3f vec=cloud[i].toVec3f();
//        for(int j=0;j<3;j++)
//            v.at<float>(j)=vec[j];

//        v.at<float>(3)=1;
//        cv::Mat transformed_point=T*v;
//        cloud[i]=cv::Vec3f(transformed_point.rowRange(0,3));
//    }
//}

cv::Vec3d find_plane(const vector<cv::Point3f> &point_cloud){//returns only a,b,c in ax+by+cz+d=0. d is assumed to be 1.
    cv::Mat m(0,3,CV_64FC1);
    for(const cv::Point3f &p:point_cloud)
        m.push_back(cv::Point3d(p));
    cv::Mat sums;
    cv::reduce(m,sums,0,cv::REDUCE_SUM);
    cv::Mat coeffs=(m.t()*m).inv()*sums.t();
    return coeffs;
}

PointCloud subsamplePointcloud(const PointCloud &in_cloud, float radius, unsigned int seed){
    PointCloud downsampled;
    cout<<"Building the tree... ";
    picoflann::KdTreeIndex<3,picoflann_point3f_adaptor> tree;
    tree.build(in_cloud.points);
    cout<<"Done!"<<endl;
    set<size_t> choosables;
    for(size_t i=0;i<in_cloud.size();i++)
        choosables.insert(i);

    uniform_int_distribution<size_t> int_dist(0,in_cloud.size()-1);
    mt19937_64 rand_generator(seed);
    cout<<"subsampleling... ";
    while(!choosables.empty()){

        size_t index=int_dist(rand_generator);
        if(choosables.count(index)>0){
            downsampled.points.push_back(in_cloud.points[index]);
            downsampled.colors.push_back(in_cloud.colors[index]);
            downsampled.normals.push_back(in_cloud.normals[index]);
            vector<pair<uint32_t,double>> points=tree.radiusSearch(in_cloud.points,in_cloud.points[index],radius);
            for(size_t i=0;i<points.size();i++){//remove the points with the radius from the point cloud
                if(choosables.count(points[i].first))
                    choosables.erase(choosables.find(points[i].first));
            }
        }
    }
    cout<<"finished subsampling..."<<endl;
    return downsampled;
}

//bool check_depthmap_point(const cv::Mat &depthmap, cv::Point position){
//    if(position.x<0 || position.x>=depthmap.cols || position.cy<0 || position.cy>=depthmap.rows)//range check
//        return false;
//    if(std::is)
//}

cv::Point3f get_depthmap_point(const cv::Mat &depthmap, const int cx, const int cy){
    bool point_found=false;
    cv::Point3f point(NAN,NAN,NAN);
    if(cx<0 || cx>=depthmap.cols || cy<0 || cy>=depthmap.rows){//range check
        point=depthmap.at<cv::Vec3f>(cy,cx);
        if(!std::isnan(point.x) && point!=cv::Point3f(0,0,0))//invalid pixel
            point_found=true;
    }
    if(!point_found){
        const int radius=5;
        cv::Point3f sum_points(0,0,0);
        int num_points=0;
        vector<cv::Point3f> points;
        for(int dx=-radius;dx<=radius;dx++){
            int x=cx+dx;
            for(int dy=-radius;dy<=radius;dy++){
                int y=cy+dy;
                if(x<0 || x>=depthmap.cols || y<0 || y>=depthmap.rows)//if out of range
                    continue;
                cv::Point3f p=depthmap.at<cv::Vec3f>(y,x);
                if(std::isnan(p.x) || p.z==0)//invalid pixel
                    continue;
                sum_points += p;
                points.push_back(p);
                num_points++;
            }
        }
        if(num_points>0){
            cv::Point3f average=sum_points/num_points;
            point = average;
            for(cv::Point3f &p:points){//show the outliers
                double dist=cv::norm(p-average);
                if(dist>0.6){
                    point=p;
                }
            }
        }
    }

    return point;
}

void get_3d_marker_corners(const vector<cv::Mat> &keyframe_depthmaps,const map<int,map<int,vector<cv::Point2f>>> &marker_frame_2d_corners, map<int,map<int,vector<cv::Point3f>>> &marker_frame_3d_corners){
    size_t i=0;
    for(const std::pair<const unsigned int,unsigned int> &f2kfi_pair:frame2keyframe_index){
        int frame_index=f2kfi_pair.first;
        const cv::Mat &keyframe_depthmap=keyframe_depthmaps[i++];
        for(const pair<const int,map<int,vector<cv::Point2f>>> &mf2c_pair:marker_frame_2d_corners){
            if(mf2c_pair.second.count(frame_index)==0)
                continue;
            int marker_id=mf2c_pair.first;
            const vector<cv::Point2f> &corners_2d=mf2c_pair.second.at(frame_index);
            vector<cv::Point3f> &corners_3d=marker_frame_3d_corners[marker_id][frame_index];
            corners_3d.resize(corners_2d.size());
            for(size_t c=0;c<corners_2d.size();c++)
                corners_3d[c]=get_depthmap_point(keyframe_depthmap, std::round(corners_2d[c].x), std::round(corners_2d[c].y));
        }
    }
}

void print_usage() {
    cout<<"Args: <key_frames_map_file> <path_to_dataset> <dataset_type> <ref_marker_id> <marker_size> [--calib <calib_path>] [--all-clouds] [--point2plane] [--neighbours-only] [--orbslam2]"<<endl;
}

int main(int argc, char *argv[])
{
    if(argc<6) {
        print_usage();
        return -1;
    }
    bool combine_clouds=false;
    bool orbslam2=false;
    bool all_clouds=false;
    bool point2plane=false;
    bool neighbours_only=false;
    bool orig=false;
    int ref_marker_id=stoi(argv[4]);
    float marker_size=stof(argv[5]);
    string calib_path;

    for(int i=6;i<argc;i++){
        string option(argv[i]);
        if(option=="--orbslam2")
            orbslam2=true;
        else if(option=="--all-clouds")
            all_clouds=true;
        else if(option=="--point2plane")
            point2plane=true;
        else if(option=="--neighbours-only")
            neighbours_only=true;
        else if(option=="--orig")
            orig=true;
        else if(option=="--calib"){
            i++;
            if(i<argc)
                calib_path=argv[i];
        }
    }

    if(all_clouds)
        cout<<"all clouds!"<<endl;
    if(point2plane)
        cout<<"point2plane!"<<endl;

    string map_path(argv[1]);
    string output_folder=map_path.substr(0,map_path.find_last_of('.'))+"_results";
    system(("mkdir "+output_folder).c_str());
    system(("rm "+output_folder+"/*.pcd").c_str());
    system(("rm "+output_folder+"/*.ply").c_str());

    shared_ptr<RGBDReader> rgbd_reader=RGBDReaderFactory::getReader(argv[3],argv[2]);
    rgbd_reader->readCalib(calib_path);

    reslam::Map m;
    m.readFromFile(map_path);

    vector<PointCloud> keyframe_clouds, smooth_clouds;
    vector<PointGrid> keyframe_point_grids;
    vector<cv::Mat> keyframe_grid_mats;
    vector<map<int,cv::Mat>> keyframe_marker_poses;

    cout<<"num_keyframes:"<<m.keyframes.size()<<endl;


    get_keyframe_transforms(m);

    map<int,map<int,vector<cv::Point2f>>> marker_frame_2d_corners;
    map<int,map<int,vector<cv::Point3f>>> marker_frame_3d_corners;
    for(auto &keyframe:m.keyframes)
        if(keyframe_transforms.count(keyframe.idx)==1){//if the key frame has a transformation add the frame and keyframe indices to the maps
            frame2keyframe_index[keyframe.fseq_idx]=keyframe.idx;
            keyframe2frame_index[keyframe.idx]=keyframe.fseq_idx;
            for(reslam::MarkerObservation &mo:keyframe.markers)//get 2d marker corners from all keyframes
                marker_frame_2d_corners[mo.id][keyframe.fseq_idx]=mo.corners;
        }


    vector<int> frame_numbers;
    for(pair<unsigned const int, unsigned int> &f2ki:frame2keyframe_index)
        frame_numbers.push_back(f2ki.first);

    read_keyframe_pointclouds(rgbd_reader, keyframe_point_grids, 1.5);//it also transforms the clouds according the map keyframe transforms
    for(PointGrid &grid:keyframe_point_grids){
        keyframe_grid_mats.push_back(grid.getGridMat());
        //replace the normals with smoothed cloud normals
        PointGrid smooth_grid=grid;//.getSmoothGrid(11);
        smooth_grid.calcNormals();
        grid.setNormalsMat(smooth_grid.getNormalsMat());
        //
        keyframe_clouds.push_back(grid.getPointCloud());
    }

    get_3d_marker_corners(keyframe_grid_mats,marker_frame_2d_corners,marker_frame_3d_corners);

    cv::Mat global2ref_T;
    vector<cv::Point3f> ref_marker_corners(4);
    float half_size=marker_size/2;
    ref_marker_corners[0]=cv::Point3f(-half_size,half_size,0);
    ref_marker_corners[1]=cv::Point3f(half_size,half_size,0);
    ref_marker_corners[2]=cv::Point3f(half_size,-half_size,0);
    ref_marker_corners[3]=cv::Point3f(-half_size,-half_size,0);

    global2ref_T=reslam::rigidBodyTransformation_Horn1987((*marker_frame_3d_corners.at(ref_marker_id).begin()).second,ref_marker_corners,true);

//    for(PointCloud &pc:smooth_clouds){//remove the plane
//        PointCloud pc_t=pc.transform(global2ref_T);
//        pc.clear();
//        for(size_t i=0;i<pc_t.points.size();i++){
//            if(pc_t.points[i].z>0.1){
//                pc.points.push_back(pc_t.points[i]);
//                pc.colors.push_back(pc_t.colors[i]);
//                pc.normals.push_back(pc_t.normals[i]);
//            }
//        }
//    }

    cout<<"writing the original clouds.."<<endl;
    for(size_t i=0;i<keyframe_clouds.size();i++)
        if(keyframe_clouds[i].size()){
            string path_prefix=output_folder+"/orig_cloud_"+to_string(frame_numbers[i]);
            PCD_IO::write_pcd(path_prefix+".pcd",keyframe_clouds[i]);
            PCD_IO::write_ply(path_prefix+".ply",keyframe_clouds[i]);
        }

    cout<<"initializing the gicp.."<<endl;
    Global_ICP gicp(keyframe_clouds,0.02);

//    const int num_iterations=20;//mannequin
    const int num_iterations=50;//human
    const pair<double,double> radius_interval(0.005,0.05);
    cout<<"executing gicp..."<<endl;
    gicp.execute(radius_interval,num_iterations,point2plane,neighbours_only,all_clouds);
    vector<cv::Mat> gicp_transforms=gicp.getFinalTransforms();

    cout<<"getting the transformed clouds..."<<endl;
    vector<PointCloud> gicp_clouds;
    if(orig)
        gicp_clouds=keyframe_clouds;
    else
        gicp_clouds=PointCloud::transformClouds(keyframe_clouds,gicp_transforms);

//    if(!gicp_clouds[0].normals.empty())
//        cout<<"Has normals!**************"<<endl;

//    return 0;

    ofstream frames_file(output_folder+"/frames.txt");
    if(gicp_clouds.size() != frame_numbers.size())
        throw runtime_error("Something is wrong!");
    for(size_t i=0;i<gicp_clouds.size();i++)
        if(gicp_clouds[i].size()){
            int frame_index=frame_numbers[i];
            frames_file<<frame_index<<endl;
            string path_prefix=output_folder+"/gicp_cloud_"+to_string(frame_index);
            PCD_IO::write_pcd(path_prefix+".pcd",gicp_clouds[i]);
            PCD_IO::write_ply(path_prefix+".ply",gicp_clouds[i]);
        }
    frames_file.close();

    std::random_device rd;
    PointCloud out_pc;
    if(combine_clouds){
        out_pc=PointCloud::combineClouds(gicp_clouds);
        //out_pc=subsamplePointcloud(out_pc,0.0019,rd());
        out_pc=subsamplePointcloud(out_pc,0.0015,rd());
        PCD_IO::write_ply(output_folder+"/combined.ply",out_pc);
    }

    map<unsigned int,cv::Mat> frame_transforms;
    size_t index=0;
    for(const std::pair<const unsigned int,unsigned int> &f2kfi_pair:frame2keyframe_index){
        frame_transforms[f2kfi_pair.first]=gicp_transforms[index++];
    }

    //transform marker corners using global icp transformations and get their average accross keyframes
    map<int,vector<cv::Point3f>> marker_average_3d_corners;
    for(pair<const int,map<int,vector<cv::Point3f>>> &mf3c_pair:marker_frame_3d_corners){
        int marker_id=mf3c_pair.first;
        map<int,vector<cv::Point3f>> &frame_3d_corners=mf3c_pair.second;
        int marker_corners=4;
        vector<cv::Point3f> sum_corners(marker_corners,cv::Point3f(0,0,0));
        vector<unsigned int> num_corners(marker_corners,0);
        for(pair<const int,vector<cv::Point3f>> &f3c_pair:frame_3d_corners){
            int frame_num=f3c_pair.first;
            vector<cv::Point3f> corners=f3c_pair.second;
            ICP::transformPointcloud(corners,frame_transforms[frame_num]);
            //visualize_marker_3d(corners,out_pc.points,out_pc.colors);
            for(int i=0;i<marker_corners;i++)
                if(!std::isnan(corners[i].x)){
                    sum_corners[i] += corners[i];
                    num_corners[i] ++;
                }
        }
        marker_average_3d_corners[marker_id].resize(4);
        for(int i=0;i<4;i++)
            marker_average_3d_corners[marker_id][i]=sum_corners[i]/float(num_corners[i]);
        visualize_marker_3d(marker_average_3d_corners[marker_id],out_pc);
    }

    PCD_IO::write_pcd(output_folder+"/gicp_cloud.pcd",out_pc);

    //get the transfrmation to the reference marker
//    vector<cv::Point3f> ref_marker_corners(4);
//    float half_size=marker_size/2;
//    ref_marker_corners[0]=cv::Point3f(-half_size,half_size,0);
//    ref_marker_corners[1]=cv::Point3f(half_size,half_size,0);
//    ref_marker_corners[2]=cv::Point3f(half_size,-half_size,0);
//    ref_marker_corners[3]=cv::Point3f(-half_size,-half_size,0);

//    cv::Mat global2ref_T=reslam::rigidBodyTransformation_Horn1987(marker_average_3d_corners[ref_marker_id],ref_marker_corners,true);
//    //convert the pointclouds to height maps, then merge the heightmaps and write the result
//    const cv::Rect2f heightmap_range(-0.2,-0.2,2,1);
//    const float cell_size=0.002;
//    const float surface_thickness=0.03;
//    vector<HeightMap> heightmaps(gicp_clouds.size());
//    for(size_t i=0;i<gicp_clouds.size();i++)
//        heightmaps[i].fromPointCloud(gicp_clouds[i].transfrom(global2ref_T),heightmap_range,cell_size,surface_thickness);

//    //merge the heightmaps
//    HeightMap merged_hm=HeightMap::merge(heightmaps);

//    write_pcd(output_folder+"/height_map.pcd",merged_hm.getPointcloud());
//    merged_hm.writeToFile(output_folder+"/merged.height_map");
//    cv::imshow("image",merged_hm.map_color.t());
//    cv::waitKey();
    //

    //write the marker corners to file
    ofstream corners_file(output_folder+"/marker_corners",std::ios_base::binary);
    size_t num_markers = marker_average_3d_corners.size();
    corners_file.write((char*)&num_markers,sizeof num_markers);
    for(pair<const int,vector<cv::Point3f>> &m_average_3d_corners:marker_average_3d_corners){
        int id = m_average_3d_corners.first;
        corners_file.write((char*)&id,sizeof id);
        for(cv::Point3f &corner:m_average_3d_corners.second){
            cv::Vec3f corner_vec=corner;
            for(int i=0;i<3;i++)
                corners_file.write((char*)&corner_vec[i],sizeof(float));
        }
    }

    return 0;

//    for(size_t i=0;i<smooth_depthmaps.size();i++){
//        cout<<"Cloud number: "<<i<<endl;
//        depthmaps::PointCloud smooth_pointcloud(keyframe_depthmaps[i]);
//        if(reconstruction.size()>1 && smooth_pointcloud.size()>1){
//            //if(!T.empty())
//            //    transform_pointcloud(smooth_pointcloud,T);

//            icp.run(smooth_pointcloud,reconstruction,1,0.05,T);
//            transform_pointcloud(smooth_pointcloud,T);
//        }
//        //for(size_t j=0;j<transformed_pointcloud.size();j++)
//        reconstruction.insert(reconstruction.end(),smooth_pointcloud.begin(),smooth_pointcloud.end());
//    }

//    reconstruction.saveToPCD(output_folder+"registered_icp_smooth.pcd");
//    vector<cv::Point3f> registered_clouds(reconstruction.size());

//    for(int i=0;i<reconstruction.size();i++)
//        registered_clouds[i]=reconstruction[i].toPoint3f();


//    vector<cv::Vec3b> registered_clouds_colors(reconstruction.size());
//    size_t index=0;
//    for(int i=0;i<keyframe_cloud_colors.size();i++)
//        for(int j=0;j<keyframe_cloud_colors[i].size();j++)
//            registered_clouds_colors[index++]=keyframe_cloud_colors[i][j];

//    write_pcd(output_folder+"registered_icp_smooth.pcd",registered_clouds,registered_clouds_colors);

    return 0;

    PointCloud transforms_pc;
    getTransformsPointCloud(rgbd_reader,transforms_pc,keyframe_transforms);
    draw_keypoint_lines(transforms_pc);
    PCD_IO::write_pcd(output_folder+"map_transforms.pcd",transforms_pc);

//    SequenceOptimizer so(m);
//    so.getTransforms(initial_frame_transforms);
//    so.optimize();
//    so.getTransforms(optimized_frame_transforms);

//    for(auto transform:initial_frame_transforms){
//        cout<<"initial: "<<transform.second<<endl;
//        cout<<"optimized: "<<optimized_frame_transforms[transform.first]<<endl;
//        cout<<"----------"<<endl;
//    }

    unsigned int io_index=0;
    for(auto &keyframe_transform:keyframe_transforms){
        keyframe_io_index[keyframe_transform.first]=io_index;
        io_index+=6;
    }

    //optimization----------
    reslam::SparseLevMarq<double>::eVector io_vec;
    reslam::SparseLevMarq<double> solver;
    reslam::SparseLevMarq<double>::Params p;
    p.verbose=true;
    p.min_average_step_error_diff=0.0000001;
    solver.setParams(p);
    keyframe_transforms_to_io_vec(io_vec);
    solver.solve(io_vec,bind(error_function,placeholders::_1,placeholders::_2));
    io_vec_to_keyframe_transforms(io_vec);
    //optimization----------

    PointCloud pc;
    getTransformsPointCloud(rgbd_reader,pc,keyframe_transforms);
    draw_keypoint_lines(pc);
    PCD_IO::write_pcd(output_folder+"keypoints_optimized.pcd",pc);


    for(auto &keyframe_transform:keyframe_transforms){
        unsigned int keyframe_index=keyframe_transform.first;
        cv::Mat transform=keyframe_transform.second;
        pc.clear();
        unsigned int frame_index=m.keyframes[keyframe_index].fseq_idx;
        rgbd_reader->readFrameAt(frame_index);
        //rgbd_reader->getCurrCloud(points,colors,transform);
        cv::Mat xyz_image,color;
        rgbd_reader->getXYZImage(xyz_image,transform);
        rgbd_reader->getCurrColor(color);
        char frame_num[10];
        std::sprintf(frame_num,"%04d",keyframe_index);
        PCD_IO::write_pcd("/media/hamid/Data/frames/"+string(frame_num)+".pcd",xyz_image,color);
    }


    return 0;
}
