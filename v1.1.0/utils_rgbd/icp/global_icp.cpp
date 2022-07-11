#include "global_icp.h"
#include "basictypes/misc.h"
#include "icp.h"
#include <set>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>

using namespace std;

void Global_ICP::get_could_means(){
    cloud_means.resize(clouds.size());
#pragma omp parallel for
    for(size_t i=0;i<clouds.size();i++){
        cloud_means[i]=cv::Point3f(0,0,0);
        for(size_t j=0;j<clouds[i].size();j++)
            cloud_means[i]+=cv::Point3d(clouds[i].points[j]);
        cloud_means[i]/=double(clouds[i].size());
    }
}

Global_ICP::Global_ICP(const std::vector<cv::Mat>  &in_depthmaps, float subsample_radius)
{
    clouds.resize(in_depthmaps.size());
//#pragma omp parallel for
    for(size_t i=0;i<in_depthmaps.size();i++){
        cv::Mat depthmap;
        in_depthmaps[i].copyTo(depthmap);
        if(depthmap.depth() != CV_32F && depthmap.depth() != CV_64F)
            throw runtime_error("The depthmap matrices need to be of a floating point format.");
        if(depthmap.channels() < 3 || depthmap.channels() > 4)
            throw runtime_error("The dephtmap matrices either 3 or 4 channels.");
        if(depthmap.depth() == CV_32F){
            if(depthmap.channels() == 3)
                subsampleDepthmap2Pointcloud<cv::Vec3f>(depthmap,clouds[i].points,subsample_radius);
            else
                subsampleDepthmap2Pointcloud<cv::Vec4f>(depthmap,clouds[i].points,subsample_radius);
        }
        if(depthmap.depth() == CV_64F){
            if(depthmap.channels() == 3)
                subsampleDepthmap2Pointcloud<cv::Vec3d>(depthmap,clouds[i].points,subsample_radius);
            else
                subsampleDepthmap2Pointcloud<cv::Vec4d>(depthmap,clouds[i].points,subsample_radius);
        }
    }
    initialize(subsample_radius);
}

template<typename T>
double distance(T &val1, T &val2){
    double diff_x=val1[0]-val2[0];
    double diff_y=val1[1]-val2[1];
    double diff_z=val1[2]-val2[2];
    return std::sqrt(diff_x*diff_x+diff_y*diff_y+diff_z*diff_z);
}

template<typename T>
void Global_ICP::subsampleDepthmap2Pointcloud(cv::Mat &depthmap,std::vector<cv::Point3f> &cloud, float subsample_radius){
    //create arrays for fast access
    vector<T*> fast_access_mat(depthmap.rows);
    for(int r=0;r<depthmap.rows;r++)
        fast_access_mat[r]=depthmap.ptr<T>(r);
    //extract points to put in the point cloud
    T last_point_picked(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max());
    for(int r=0;r<depthmap.rows;r++){
        //bool is_last_row = (r==depthmap.rows-1);
        for(int c=0;c<depthmap.cols;c++){
            T &val=fast_access_mat[r][c];
            //ignore the pixel if it is invalid
            if(std::isnan(val[0]))
                continue;
            if(depthmap.channels()>3){
                int64_t last_channel=0;
                if(depthmap.depth() == CV_32F)
                    last_channel=*((int32_t*)&val[3]);
                else if(depthmap.depth() == CV_64F)
                    last_channel=*((int64_t*)&val[3]);
                if((last_channel & 1) == 0)
                    continue;
            }
            if(distance(val,last_point_picked)>subsample_radius){//add the pixel to the point cloud
                cloud.push_back(cv::Point3f(val[0],val[1],val[2]));
                for(int i=0;i<3;i++)
                    last_point_picked[i]=val[i];
            }
        }
    }
}

Global_ICP::Global_ICP(const std::vector<PointCloud> &in_clouds, float subsample_radius)
{
    clouds=in_clouds;
    initialize(subsample_radius);
}

void Global_ICP::initialize(float subsample_radius){
    num_clouds=clouds.size();
    trees.resize(num_clouds);
    transforms.resize(num_clouds);
    get_could_means();
    cout<<"subsampling the clouds...";
    random_device rand_d;
    vector<unsigned int> seeds(num_clouds);
    for(size_t i=0;i<num_clouds;i++)
        seeds[i]=rand_d();
#pragma omp parallel for
    for(size_t i=0;i<num_clouds;i++)
        clouds[i]=ICP::subsamplePointCloud(clouds[i],subsample_radius,seeds[i]);
    cout<<"done!"<<endl;
}

void Global_ICP::execute(pair<double,double> radius_range, int num_iterations, bool point_to_plane, bool neighbours_only, bool all_clouds){
    pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZINormal,pcl::PointXYZINormal> point_to_plane_r;

    std::vector<PointCloud> curr_clouds=clouds;

    for(size_t c=0;c<transforms.size();c++)
        transforms[c].resize(num_iterations);

    double radius_interval_size=radius_range.second-radius_range.first;

    for(int i=0;i<num_iterations;i++){
        cout<<"gicp iteration: "<<i<<endl;
        cout<<"Building the KDtrees.."<<endl;

#pragma omp parallel for
        for(size_t i=0;i<num_clouds;i++)
            trees[i].build(curr_clouds[i].points);
        cout<<"Done!"<<endl;

        double radius=(radius_interval_size*(num_iterations-i))/num_iterations+radius_range.first;
        cout<<radius<<endl;
#pragma omp parallel for
        for(size_t c=0;c<curr_clouds.size();c++){//loop over clouds to find the transforms
            transforms[c][i]=cv::Mat::eye(4,4,CV_32FC1);
            if(curr_clouds[c].empty())
                continue;
            //vector<cv::Point3f> source,target;
            pcl::PointCloud<pcl::PointXYZINormal> source,target;
            vector<cv::Point3f> src,trg;
            for(size_t p=0;p<curr_clouds[c].size();p++){//loop over points
                vector<tuple<cv::Point3f,cv::Vec3f,double,int>> closest_points;
                for(size_t t=0;t<trees.size();t++){//loop over trees
                    if(curr_clouds[t].empty() || t==c)
                        continue;
                    if(neighbours_only && std::abs((long long)t-(long long)c)>1)
                        continue;
                    vector<pair<uint32_t,double>> result=trees[t].radiusSearch(curr_clouds[t].points,curr_clouds[c].points[p],radius);
                    if(result.size()>0){
                        double distance=result[0].second;
                        size_t index=result[0].first;
                        std::tuple<cv::Point3f,cv::Vec3f,double,int> closest_point(curr_clouds[t].points[index],curr_clouds[t].normals[index],distance,t);
                        closest_points.push_back(closest_point);
                    }
                }
                if(!closest_points.empty()){
                    if(all_clouds){
                        for(tuple<cv::Point3f,cv::Vec3f,double,int> &cp:closest_points){
                            if(point_to_plane){
                                target.push_back(ICP::getPCLPoint(get<0>(cp),get<1>(cp)));
                                source.push_back(ICP::getPCLPoint(curr_clouds[c].points[p],curr_clouds[c].normals[p]));
                            }
                            else{
                                trg.push_back(get<0>(cp));
                                src.push_back(curr_clouds[c].points[p]);
                            }
                        }
                    }
                    else{//get the closest point in any cloud
                        cv::Point3f best_point;
                        cv::Vec3f best_point_normal;
                        double best_dist=numeric_limits<double>::max();
                        int best_cloud_index=-1;
                        for(tuple<cv::Point3f,cv::Vec3f,double,int> &cp:closest_points){
                            if(best_dist>get<2>(cp)){
                                best_point=get<0>(cp);
                                best_point_normal=get<1>(cp);
                                best_dist=get<2>(cp);
                                best_cloud_index=get<3>(cp);
                            }
                        }
//                        double cloud_distance=cv::norm(cloud_means[best_cloud_index]-cloud_means[c]);
//                        weight = pow(4,-cloud_distance/*-std::abs(best_cloud_index-int(c))*/);
//                        weight = 1.0;
                        if(point_to_plane){
                            target.push_back(ICP::getPCLPoint(best_point,best_point_normal));
                            source.push_back(ICP::getPCLPoint(curr_clouds[c].points[p],curr_clouds[c].normals[p]));
                        }
                        else{
                            trg.push_back(best_point);
                            src.push_back(curr_clouds[c].points[p]);
                        }
                    }
                }
            }

            if(point_to_plane && source.size()>0){
                pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZINormal,pcl::PointXYZINormal>::Matrix4 pcl_mat;
                point_to_plane_r.estimateRigidTransformation(source,target,pcl_mat);
                for(int j=0;j<3;j++)
                    for(int k=0;k<4;k++)
                        transforms[c][i].at<float>(j,k)=pcl_mat.coeff(j,k);
            }
            else if(src.size()>0)// point2point
                transforms[c][i]=ucoslam::rigidBodyTransformation_Horn1987(src,trg,true);

        }
        for(size_t c=0;c<curr_clouds.size();c++){//loop over clouds to apply the transforms
            if(curr_clouds[c].empty())
                continue;
            curr_clouds[c]=curr_clouds[c].transform(transforms[c][i]);
            cv::Matx33d R;
            cv::Point3d t;
            ICP::get_Rt(transforms[c][i],R,t);
            cloud_means[c]=R*cloud_means[c]+t;
        }
    }
}

vector<vector<cv::Point3f>> Global_ICP::getClouds(const std::vector<std::vector<cv::Point3f> > &in_clouds, int num_iterations){
    vector<vector<cv::Point3f>> transformed_clouds=in_clouds;
    const vector<cv::Mat> &final_transforms=Global_ICP::getFinalTransforms(num_iterations);
    cout<<"Transforming the clouds... ";
#pragma omp parallel for
    for(size_t c=0;c<final_transforms.size();c++)
        ICP::transformPointcloud(transformed_clouds[c],final_transforms[c]);
    cout<<"Done"<<endl;
    return transformed_clouds;
}

vector<cv::Mat> Global_ICP::getFinalTransforms(int num_iterations){
    vector<cv::Mat> final_transforms(transforms.size());
    if(num_iterations==-1 && transforms.size()>0)
        num_iterations=transforms[0].size();
    cout<<"Calculating the final transforms... ";
#pragma omp parallel for
    for(size_t c=0;c<transforms.size();c++){
        cv::Mat transform=cv::Mat::eye(4,4,CV_64FC1);
        //get the final transform
        for(size_t i=0;i<num_iterations;i++){
            cv::Mat T;
            transforms[c][i].convertTo(T,CV_64FC1);
            transform=T*transform;
        }
        final_transforms[c]=transform;
    }
    cout<<"Done"<<endl;
    return final_transforms;
}

vector<vector<cv::Mat>> Global_ICP::getTransforms(){
    return transforms;
}

//int Global_ICP::importance(double distance,double max_distance,int iteration,int num_iterations){
//    double mu=(max_distance*(num_iterations-1-iteration))/(num_iterations-1);
//    double min_value;
//    if(mu<max_distance/2)
//        min_value=importance_func(max_distance,mu,max_distance);
//    else
//        min_value=importance_func(0,mu,max_distance);
//    double value=importance_func(distance,mu,max_distance);
//    return std::round(value/min_value);
//}


