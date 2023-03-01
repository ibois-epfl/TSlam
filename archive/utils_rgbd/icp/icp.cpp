#include <set>
#include <iostream>
#include "icp.h"
#include "basictypes/misc.h"
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>

using namespace std;

ICP::ICP(const vector<cv::Point3f> &src, const vector<cv::Point3f> &dst, float ss_radius, const vector<cv::Vec3f> &src_normals, const vector<cv::Vec3f> &dst_normals)
{
    PointCloud src_cloud,dst_cloud;
    src_cloud.points=src;
    src_cloud.normals=src_normals;
    dst_cloud.points=dst;
    dst_cloud.normals=dst_normals;
    init(src_cloud,dst_cloud,ss_radius);
}

ICP::ICP(const PointCloud &src, const PointCloud &dst, float ss_radius){
    init(src,dst,ss_radius);
}

void ICP::init(const PointCloud &src, const PointCloud &dst, float ss_radius){
    if(ss_radius>0){
        source=subsamplePointCloud(src,ss_radius,std::rand());
        dest=subsamplePointCloud(dst,ss_radius,std::rand());
    }
    else{
        source=src;
        dest=dst;
    }
}

pcl::PointXYZINormal ICP::getPCLPoint(const cv::Point3f &point, const cv::Vec3f &normal){
    pcl::PointXYZINormal output;
    output.x=point.x;
    output.y=point.y;
    output.z=point.z;
    for(int i=0;i<3;i++)
        output.normal[i]=normal[i];
    return output;
}

cv::Mat ICP::exec(pair<double,double> radius_range, int num_iterations, bool point_to_plane){
    pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZINormal,pcl::PointXYZINormal> point_to_plane_r;
    point_to_plane = point_to_plane && !source.normals.empty() && !dest.normals.empty();
    double radius_interval_size=radius_range.second-radius_range.first;
    picoflann::KdTreeIndex<3,picoflann_point3f_adaptor> tree;
    tree.build(dest.points);
    cv::Mat transform=cv::Mat::eye(4,4,CV_32FC1);
    if(point_to_plane)
        cout<<"point2plane"<<endl;
    else
        cout<<"point2point"<<endl;

    for(int i=0;i<num_iterations;i++){

        //find correspondences
        double radius=(radius_interval_size*(num_iterations-i))/num_iterations+radius_range.first;
        cout<<radius<<endl;
        vector<cv::Point3f> dst_points,src_points;
        pcl::PointCloud<pcl::PointXYZINormal> pcl_src_points, pcl_dst_points;
        for(size_t p=0;p<source.size();p++){
            vector<pair<uint32_t,double>> result=tree.radiusSearch(dest.points,source.points[p],radius);
            if(!result.empty())
                if(point_to_plane){
                    pcl_dst_points.push_back(getPCLPoint(dest.points[result[0].first],dest.normals[result[0].first]));
                    pcl_src_points.push_back(getPCLPoint(source.points[p],dest.normals[p]));
                }
                else{
                    dst_points.push_back(dest.points[result[0].first]);
                    src_points.push_back(source.points[p]);
                }
        }
        cv::Mat curr_transform=cv::Mat::eye(4,4,CV_32FC1);
        if(point_to_plane && pcl_dst_points.size()>0){
            pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZINormal,pcl::PointXYZINormal>::Matrix4 pcl_mat;
            point_to_plane_r.estimateRigidTransformation(pcl_src_points,pcl_dst_points,pcl_mat);
            for(int j=0;j<3;j++)
                for(int k=0;k<4;k++)
                    curr_transform.at<float>(j,k)=pcl_mat.coeff(j,k);
        }
        else if(!point_to_plane && dst_points.size()>0)
            curr_transform=tslam::rigidBodyTransformation_Horn1987(src_points,dst_points,true);
        source=source.transform(curr_transform);
        transform=curr_transform*transform;
    }
    return transform;
}

void ICP::get_Rt(const cv::Mat T, cv::Matx33d &R, cv::Point3d &t){
    cv::Mat T_64;
    T.convertTo(T_64,CV_64FC1);
    R=cv::Matx33d(T_64(cv::Range(0,3),cv::Range(0,3)));
    t=cv::Vec3d(T_64(cv::Range(0,3),cv::Range(3,4)));
}

void ICP::get_Rt(const cv::Mat T, cv::Matx33f &R, cv::Point3f &t){
    cv::Mat T_32;
    T.convertTo(T_32,CV_32FC1);
    R=cv::Matx33f(T_32(cv::Range(0,3),cv::Range(0,3)));
    t=cv::Vec3f(T_32(cv::Range(0,3),cv::Range(3,4)));
}


void ICP::transformPointcloud(vector<cv::Point3f> &cloud, const cv::Mat T){
    cv::Matx33f R;
    cv::Point3f t;
    get_Rt(T,R,t);

    for(cv::Point3f &p:cloud)
        p=R*p+t;
}

vector<cv::Point3f> ICP::subsamplePointCloud(const vector<cv::Point3f> &in_cloud, float radius, unsigned int seed){
    PointCloud input;
    input.points=in_cloud;
    PointCloud downsampled=subsamplePointCloud(input, radius, seed);
    return downsampled.points;
}

PointCloud ICP::subsamplePointCloud(const PointCloud &in_cloud, float radius, unsigned int seed){
    PointCloud downsampled;

    picoflann::KdTreeIndex<3,picoflann_point3f_adaptor> tree;
    tree.build(in_cloud.points);

    set<size_t> choosables;
    for(size_t i=0;i<in_cloud.size();i++)
        choosables.insert(i);

    uniform_int_distribution<size_t> int_dist(0,in_cloud.size()-1);
    mt19937_64 rand_generator(seed);

    while(!choosables.empty()){

        size_t index=int_dist(rand_generator);
        if(choosables.count(index)>0){
            downsampled.points.push_back(in_cloud.points[index]);
            if(!in_cloud.normals.empty())
                downsampled.normals.push_back(in_cloud.normals[index]);
            if(!in_cloud.colors.empty())
                downsampled.colors.push_back(in_cloud.colors[index]);
            vector<pair<uint32_t,double>> points=tree.radiusSearch(in_cloud.points,in_cloud.points[index],radius);
            for(size_t i=0;i<points.size();i++){
                if(choosables.count(points[i].first))
                    choosables.erase(choosables.find(points[i].first));
            }
        }

    }

    return downsampled;
}
