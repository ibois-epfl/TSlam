#include<iostream>
#include<fstream>
#include<sstream>
#include<map>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include<pointcloud.h>
#include<limits>
//#include<depthmaps/depthmaputils.h>
#include "icp.h"
#include "basictypes/misc.h"
#include "heightmap.h"
#include "pcd_io.h"

using namespace std;

map<int,vector<cv::Point3f>> read_markers_corners(string path){
    ifstream in_file(path,ios_base::binary);
    if(!in_file.is_open())
        throw runtime_error("Could not open a markers file at: "+path);

    map<int,vector<cv::Point3f>> marker_corners;

    size_t num_markers;
    in_file.read((char*)&num_markers,sizeof num_markers);
    for(size_t m=0;m<num_markers;m++){
        int id;
        in_file.read((char*)&id,sizeof id);
        marker_corners[id].resize(4);
        vector<cv::Point3f> &corners=marker_corners[id];
        cout<<"Marker "<<id<<" : ";
        for(int c=0;c<4;c++){
            cv::Vec3f corner;
            for(int d=0;d<3;d++)
                in_file.read((char*)&corner[d],sizeof(float));
            corners[c]=corner;
            cout<<corners[c]<<" ";
        }
        cout<<endl;
    }
    cout<<num_markers<<" markers were read."<<endl;
    return marker_corners;
}

vector<PointCloud> read_folder_clouds(string folder_path){
    ifstream frames_file(folder_path+"/frames.txt");
    if(!frames_file.is_open())
        throw runtime_error("Could not open the frames.txt file in folder:"+folder_path);

    //read the frame indices
    vector<int> frame_indices;
    int frame_index;
    while(frames_file>>frame_index)
        frame_indices.push_back(frame_index);

    vector<PointCloud> output(frame_indices.size());

    for(size_t i=0;i<frame_indices.size();i++)
        PCD_IO::read_pcd(folder_path+"/gicp_cloud_"+to_string(frame_indices[i])+".pcd",output[i].points,&output[i].colors);

    return output;
}

//void heightmap2depthmap(const HeightMap &hm, depthmaps::DepthMap &depth_map){
//    depth_map.create(map_mat.rows,map_mat.cols,true);
//    for(int r=0;r<map_mat.rows;r++){
//        depthmaps::DepthPixel *dm_row=depth_map.ptr<depthmaps::DepthPixel>(r);
//        const float *map_mat_row=map_mat.ptr<float>(r);
//        const cv::Vec3b *image_row=image.ptr<cv::Vec3b>(r);
//        for(int c=0;c<map_mat.cols;c++){
//            //set the point
//            dm_row[c].set(c,r,map_mat_row[c]);
//            //set the color
//            const cv::Vec3b &color=image_row[c];
//            dm_row[c].setRGB(color[2],color[1],color[0]);
//        }
//    }
//}

void print_usage(){
    cout<<"Args: <ref_cloud_folder> <ref_marker_id> <marker_size> [<source_cloud_folder>]"<<endl;
}

void call_back_mouse(int event, int x, int y, int, void* data){
    vector<cv::Point> *points=(vector<cv::Point>*)data;
    if(event == cv::EVENT_LBUTTONUP)
        points->push_back(cv::Point(x,y));
    else if(event == cv::EVENT_RBUTTONUP)
        points->pop_back();
    else if(event == cv::EVENT_MBUTTONUP)
        points->push_back(cv::Point(-1,-1));
}

void get_click_positions(cv::Mat image, string window_name, vector<cv::Point> &mouse_coords, int num_points){
    mouse_coords.clear();
    cv::Mat curr_image;
    int marker_num;
    cv::imshow(window_name,image);
    cv::setMouseCallback(window_name,call_back_mouse,&mouse_coords);
    while(mouse_coords.size()<num_points+1){
        image.copyTo(curr_image);
        cv::putText(curr_image,"Current Marker Num: "+to_string(mouse_coords.size()),cv::Point(30,30),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(0,255,0));
        for(size_t i=0;i<mouse_coords.size();i++){
            cv::Point p=mouse_coords[i];
            if(p.x>=0 && p.y>=0){//if the point is not skipped draw it
                cv::Scalar red(0,0,255);
                cv::drawMarker(curr_image,p,red,cv::MARKER_STAR);
                cv::putText(curr_image,to_string(i),p,cv::FONT_HERSHEY_PLAIN,1,red);
            }
        }
        cv::imshow(window_name,curr_image);
        marker_num = cv::waitKey(100)-'0';
    }
    mouse_coords.pop_back();
}

vector<cv::Point2f> get_average_click_coords(const vector<HeightMap> &orig_height_maps, string window_name, int num_markers){
    vector<vector<cv::Point>> hm_click_coords;
    for(const HeightMap &hm:orig_height_maps){
        cv::Mat orig_hm_image=hm.getHeightImage();
        cv::Mat orig_hm_image_show;
        cv::cvtColor(orig_hm_image,orig_hm_image_show,cv::COLOR_GRAY2BGR);
        vector<cv::Point> click_coords;

        get_click_positions(orig_hm_image_show,window_name,click_coords,num_markers);
        hm_click_coords.push_back(click_coords);
    }
    //get the average position of the clicks
    vector<cv::Point2f> average_click_coords(num_markers,cv::Point2f(0,0));
    vector<int> num_click_coords(num_markers,0);
    for(vector<cv::Point>& click_coords:hm_click_coords){
        for(int i=0;i<num_markers;i++)
            if(click_coords[i].x>=0 && click_coords[i].y>=0){//if the point is not skipped add it to the sum
                average_click_coords[i]+=cv::Point2f(click_coords[i]);
                num_click_coords[i]++;
            }
    }
    for(int i=0;i<num_markers;i++)
        average_click_coords[i]/=num_click_coords[i];

    return average_click_coords;
}

void draw_3d_line_points(cv::Point3f start, const cv::Point3f end, int steps, cv::Vec3b start_color, cv::Vec3b end_color, PointCloud& pc){
    cv::Point3d step=(end-start)/steps;
    cv::Vec3d color_step=(cv::Vec3d(end_color)-cv::Vec3d(start_color))/double(steps);
    cv::Point3d point=start;
    for(int i=0; i<steps; i++,point+=step) {
        pc.points.push_back(point);
        pc.colors.push_back(start_color+cv::Vec3b(i*color_step));
        //pc.normals.emplace_back(0,0,0);
    }
}

cv::Mat alpha_mix(cv::Mat im1, cv::Mat im2, cv::Mat im2_alpha){
    std::vector<cv::Mat> alpha_channels(3);
    cv::Mat alpha;

    for(int i=0;i<3;i++)
        alpha_channels[i]=im2_alpha;

    cv::merge(alpha_channels,alpha);
    cv::Mat alpha_res=cv::Scalar(1,1,1)-alpha;

    return im1.mul(alpha_res)+im2.mul(alpha);
}

cv::Mat get_heightmap_mask(const HeightMap &height_map, const std::pair<cv::Vec3f,float> &plane_coeffs, double plane_dist_thresh){
    PointGrid pg=height_map.getPointGrid();
    cv::Mat float_mask,plane_dists=pg.getPlaneDists(plane_coeffs).t();
    cv::Mat(-plane_dists>plane_dist_thresh).convertTo(float_mask,CV_32FC1);
    return float_mask;
}

cv::Mat apply_float_mask(cv::Mat image,cv::Mat float_mask){
    cv::Mat mask=float_mask;
    if(float_mask.channels()==1 && image.channels()==3){
        std::vector<cv::Mat> color_channels(3);
        for(int i=0;i<3;i++)
            float_mask.copyTo(color_channels[i]);
        cv::merge(color_channels,mask);
    }
    return image.mul(mask/255)+(255-mask);
}

int main(int argc, char *argv[]){
    if(argc<4){
        print_usage();
        return -1;
    }
    bool use_height_image=false;
    string orig_cloud_folder=string(argv[1])+'/';
    int ref_marker=stoi(argv[2]);
    float marker_size=stof(argv[3]);
    string new_cloud_folder;
    if(argc>4)
        new_cloud_folder=string(argv[4])+'/';

    //read the marker corners in the reference frame
    string orig_markers_path=orig_cloud_folder+"marker_corners";
    const map<int,vector<cv::Point3f>> &orig_markers_corners=read_markers_corners(orig_markers_path);

    //get the transform to the reference marker in the reference frame
    cv::Mat global2ref_T;
    vector<cv::Point3f> ref_marker_corners(4);
    float half_size=marker_size/2;
    ref_marker_corners[0]=cv::Point3f(-half_size,half_size,0);
    ref_marker_corners[1]=cv::Point3f(half_size,half_size,0);
    ref_marker_corners[2]=cv::Point3f(half_size,-half_size,0);
    ref_marker_corners[3]=cv::Point3f(-half_size,-half_size,0);

    cv::Mat marker_rotation;
    double theta=std::acos(-1);
    theta=0;
    cv::Rodrigues(cv::Vec3f(0,0,theta),marker_rotation);

    global2ref_T=ucoslam::rigidBodyTransformation_Horn1987(orig_markers_corners.at(ref_marker),ref_marker_corners,true);
    global2ref_T(cv::Range(0,3),cv::Range(0,3))=marker_rotation*global2ref_T(cv::Range(0,3),cv::Range(0,3));
    cv::Matx33f R(global2ref_T(cv::Range(0,3),cv::Range(0,3)));
    cv::Vec3f t(global2ref_T(cv::Range(0,3),cv::Range(3,4)));

    vector<cv::Point3f> all_corners;
    for(const pair<int,vector<cv::Point3f>> &marker_corners:orig_markers_corners)
        all_corners.insert(all_corners.end(),marker_corners.second.begin(),marker_corners.second.end());

    //find markers' plane
    cv::Mat A(0,3,CV_32FC1),b(all_corners.size(),1,CV_32FC1,cv::Scalar(-1));
    for(cv::Point3f &corner:all_corners)
        A.push_back(cv::Mat(R*cv::Vec3f(corner)+t).t());

    cv::Vec3f plane_coeffs(cv::Mat(A.inv(cv::DECOMP_SVD)*b));
    double plane_dist_thresh=0;//kiko: 0.02

    cout<<plane_coeffs<<endl;

    cv::Rect2f hm_ranges;
    double grid_step;
    hm_ranges=cv::Rect2f(-.3,-.2,1.6,1); grid_step=0.0015;//for Kiko's sequences, marker 235 theta=0
//    hm_ranges=cv::Rect2f(-0.5,-1,1.5,0.1); grid_step=0.003;//for Kiko's sequences 320x240 (marker 248) theta=0
//    hm_ranges=cv::Rect2f(-.1,-.2,2,1); grid_step=0.0015;//for mannequin sequences, theta=0
//    hm_ranges=cv::Rect2f(-0.5,0,2,1); grid_step=0.0015;//for rafa's sequences, ref marker 0 (0.108m) theta=PI
//    hm_ranges=cv::Rect2f(0,0.1,2.1,1.2); grid_step=0.0015;//for hamid's sequences, ref marker 0 (0.108m) theta=PI


    //get the height maps from all frame clouds
    vector<PointCloud> orig_clouds=PointCloud::transformClouds(read_folder_clouds(orig_cloud_folder),global2ref_T);
    vector<HeightMap> orig_height_maps=HeightMap::fromPointClouds(orig_clouds,hm_ranges,grid_step,0.03);
    HeightMap orig_height_map=HeightMap::merge(orig_height_maps);



    if(new_cloud_folder.empty()){
        PCD_IO::write_pcd(orig_cloud_folder+"/merged_heightmap.pcd",orig_height_map.getPointcloud());
        orig_height_map.writeToFile(orig_cloud_folder+"/merged.heightmap");
        PointGrid pg=orig_height_map.getPointGrid();
        pg.calcNormals();

        cv::Mat float_mask,plane_dists=pg.getPlaneDists(std::make_pair(plane_coeffs,1)).t();
        cv::Mat(-plane_dists>plane_dist_thresh).convertTo(float_mask,CV_32FC1);

        cv::imwrite(orig_cloud_folder+"heightmap_image.png",orig_height_map.getHeightImage().mul(float_mask/255)+(255-float_mask));

        PointCloud pc=pg.getPointCloud();
        pc.reverseNormals();
        PCD_IO::write_ply(orig_cloud_folder+"/merged_heightmap.ply",pc);

        cv::Mat image;
        if(use_height_image)
            cv::cvtColor(orig_height_map.getHeightImage(),image,cv::COLOR_GRAY2BGR);
        else
            orig_height_map.getRGB().copyTo(image);

        //get 2D coordinates
        vector<cv::Point2i> click_coords;//=get_average_click_coords(orig_height_maps,"ref_scene",num_markers);
        get_click_positions(image,"height image",click_coords,8);
        vector<cv::Point3f> marker_coords_3D;
        //convert to 3D coords
        for(cv::Point2i p:click_coords){
            marker_coords_3D.push_back(orig_height_map.get3DCoords(p));
        }

        ofstream mouse_points_file(orig_cloud_folder+"/mousepoints",ios_base::binary);
        size_t num_points=marker_coords_3D.size();
        mouse_points_file.write((char*)&num_points,sizeof(size_t));
        for(cv::Point3f p:marker_coords_3D){
            mouse_points_file.write((char*)&p.x,sizeof(float));
            mouse_points_file.write((char*)&p.y,sizeof(float));
            mouse_points_file.write((char*)&p.z,sizeof(float));
        }
    }
    else{
        //read the clouds
        string orig_cloud_path=orig_cloud_folder+"gicp_cloud.pcd";
        PointCloud orig_cloud=PCD_IO::read_pcd(orig_cloud_path);
        string new_cloud_path=new_cloud_folder+"gicp_cloud.pcd";
        PointCloud new_cloud=PCD_IO::read_pcd(new_cloud_path);

        cout<<"CLOUD SIZE!: "<<new_cloud.size()<<endl;

        //read new cloud
        string new_markers_path=new_cloud_folder+"marker_corners";
        const map<int,vector<cv::Point3f>> &new_markers_corners=read_markers_corners(new_markers_path);

        //make corresponding point sets
        vector<cv::Point3f> orig_marker_points;
        vector<cv::Point3f> new_marker_points;
        for(pair<const int, vector<cv::Point3f>> rm_corners:orig_markers_corners){
            int id=rm_corners.first;
            if(new_markers_corners.count(id)){
                cout<<"marker id:"<<id<<endl;
                orig_marker_points.insert(orig_marker_points.end(),rm_corners.second.begin(),rm_corners.second.end());
                new_marker_points.insert(new_marker_points.end(),new_markers_corners.at(id).begin(),new_markers_corners.at(id).end());
            }
        }
        //get the transformation and transform the new cloud
        cv::Mat T=ucoslam::rigidBodyTransformation_Horn1987(new_marker_points,orig_marker_points,true);
        cout<<T<<endl;
        new_cloud=new_cloud.transform(T);
        cout<<"CLOUD SIZE NOW!: "<<new_cloud.size()<<endl;
        //create a combined cloud
        PointCloud combined_cloud=orig_cloud+new_cloud;
        PCD_IO::write_pcd(new_cloud_folder+"combined.pcd",new_cloud);

        //transform the coordinate systems to the reference marker
        orig_cloud.transform(global2ref_T);
        new_cloud.transform(global2ref_T);

        //color the cloud according to the error
        orig_cloud.points=ICP::subsamplePointCloud(orig_cloud.points,0.02,0);
        picoflann::KdTreeIndex<3,picoflann_point3f_adaptor> tree;
        tree.build(orig_cloud.points);

        for(size_t i=0;i<new_cloud.points.size();i++){
            cv::Point3f &point=new_cloud.points[i];
            vector<pair<uint32_t,double>> match=tree.radiusSearch(orig_cloud.points,point,0.05);
            if(!match.empty()){
                new_cloud.colors[i][0]=match[0].second/0.001*255;
                new_cloud.colors[i][1]=256-new_cloud.colors[i][0];
                new_cloud.colors[i][2]=0;
            }
        }

        PCD_IO::write_pcd(new_cloud_folder+"comparison.pcd",new_cloud);
        //get the height maps from all frame clouds
        vector<PointCloud> new_clouds=PointCloud::transformClouds(read_folder_clouds(new_cloud_folder),global2ref_T*T);
        vector<HeightMap> new_height_maps=HeightMap::fromPointClouds(new_clouds,hm_ranges,grid_step,0.03);
        HeightMap new_height_map=HeightMap::merge(new_height_maps);

        cv::Mat new_float_mask=get_heightmap_mask(new_height_map,std::make_pair(plane_coeffs,1.0f),plane_dist_thresh);
        cv::Mat new_heightmap_image=apply_float_mask(new_height_map.getHeightImage(),new_float_mask);

        cv::imwrite(new_cloud_folder+"heightmap_image.png",new_heightmap_image);
        cv::imwrite(new_cloud_folder+"orthogonal_image.png",new_height_map.map_color.t());

        vector<cv::Mat> color_channels(3);

        cv::Mat orig_heightmap_image;
        orig_heightmap_image=orig_height_map.getHeightImage();
        cv::Mat orig_float_mask=get_heightmap_mask(orig_height_map,std::make_pair(plane_coeffs,1.0f),plane_dist_thresh);
        cv::Mat float_mask; cv::max(new_float_mask,orig_float_mask,float_mask);

        cv::Mat red_image=cv::Mat(new_heightmap_image.rows,new_heightmap_image.cols,CV_32FC3,cv::Scalar(0,0,255));
        cv::Mat blue_image=cv::Mat(orig_heightmap_image.rows,orig_heightmap_image.cols,CV_32FC3,cv::Scalar(255,0,0));
        cv::Mat white_image=cv::Mat(orig_heightmap_image.rows,orig_heightmap_image.cols,CV_32FC3,cv::Scalar(255,255,255));

        cv::Mat orig_alpha=orig_heightmap_image/255.0*3/4;

//        color_channels[0]=cv::Mat(orig_heightmap_image.rows,orig_heightmap_image.cols,CV_32FC1,cv::Scalar(255));
//        color_channels[1]=255-orig_heightmap_image;
//        color_channels[2]=255-orig_heightmap_image;

        orig_heightmap_image.copyTo(color_channels[0]);
        color_channels[1]=cv::Mat::zeros(orig_heightmap_image.rows,orig_heightmap_image.cols,CV_32FC1);
        color_channels[2]=cv::Mat::zeros(orig_heightmap_image.rows,orig_heightmap_image.cols,CV_32FC1);

        cv::Mat ohi_blue;
        cv::merge(color_channels,ohi_blue);
        cv::Mat colorized_ohi=alpha_mix(white_image,/*ohi_blue*/blue_image,orig_alpha);

        new_heightmap_image=new_height_map.getHeightImage();

//        color_channels[0]=255-new_heightmap_image;
//        color_channels[1]=255-new_heightmap_image;
//        color_channels[2]=cv::Mat(new_heightmap_image.rows,new_heightmap_image.cols,CV_32FC1,cv::Scalar(255));
        color_channels[0]=cv::Mat::zeros(orig_heightmap_image.rows,orig_heightmap_image.cols,CV_32FC1);
        color_channels[1]=cv::Mat::zeros(orig_heightmap_image.rows,orig_heightmap_image.cols,CV_32FC1);
        new_heightmap_image.copyTo(color_channels[2]);

        cv::Mat new_alpha=new_heightmap_image/255.0*3/4;
        cv::Mat nhi_red;
        cv::merge(color_channels,nhi_red);
        cv::Mat combination_image=alpha_mix(colorized_ohi,/*nhi_red*/red_image,new_alpha);
        combination_image=apply_float_mask(combination_image,float_mask);
//        cv::min(colorized_nhi,colorized_ohi,combination_image);
//        combination_image=(colorized_nhi+colorized_ohi)/2.0;


        cv::imwrite(new_cloud_folder+"hm_super_imposed.png",combination_image);

        //PCD_IO::write_pcd(new_cloud_folder+"merged_heightmap.pcd",new_height_map.getPointcloud());

        PointGrid hm_pg=new_height_map.getPointGrid();
        hm_pg.calcNormals();
        PointCloud hm_pc=hm_pg.getPointCloud();
        hm_pc.reverseNormals();

        PCD_IO::write_pcd(new_cloud_folder+"merged_heightmap.pcd",hm_pc);
        PCD_IO::write_ply(new_cloud_folder+"merged_heightmap.ply",hm_pc);
        new_height_map.writeToFile(new_cloud_folder+"merged.heightmap");
        //    ofstream out_file(new_cloud_folder+"/click_transform",ios_base::binary);
        //    cv::Mat click_T=ucoslam::rigidBodyTransformation_Horn1987(new_mouse_3D_coords,orig_mouse_3D_coords,true);
        //    //write transform
        //    for(int i=0;i<3;i++)
        //        for(int j=0;j<4;j++)
        //            out_file.write((char*)&click_T.at<float>(i,j),sizeof(float));

        HeightMap diff_hm=new_height_map.getDiffFrom(orig_height_map);
        PCD_IO::write_pcd(new_cloud_folder+"diff_cloud.pcd",diff_hm.getPointcloud());
        cv::Mat diff_im=diff_hm.map_color.t();
        cv::imshow("diff",diff_im);
        cv::imwrite(new_cloud_folder+"diff.png",diff_im);
        cv::waitKey();

        cv::Mat image;
        if(use_height_image)
            cv::cvtColor(new_height_map.getHeightImage(),image,cv::COLOR_GRAY2BGR);
        else
            new_height_map.map_color.copyTo(image);
        //get 2D coordinates
        vector<cv::Point2i> click_coords;//=get_average_click_coords(new_height_maps,"ref_scene",num_markers);
        get_click_positions(image,"height",click_coords,8);
        vector<cv::Point3f> marker_coords_3D;
        //convert to 3D coords
        for(cv::Point2i p:click_coords)
            marker_coords_3D.push_back(new_height_map.get3DCoords(p));

        ofstream mouse_points_file(new_cloud_folder+"/mousepoints",ios_base::binary);
        size_t num_points=marker_coords_3D.size();
        mouse_points_file.write((char*)&num_points,sizeof(size_t));
        for(cv::Point3f p:marker_coords_3D){
            mouse_points_file.write((char*)&p.x,sizeof(float));
            mouse_points_file.write((char*)&p.y,sizeof(float));
            mouse_points_file.write((char*)&p.z,sizeof(float));
        }

    }


    return 0;
}
