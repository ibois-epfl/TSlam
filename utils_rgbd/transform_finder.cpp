#include <iostream>
#include <fstream>
#include <opencv2/calib3d.hpp>
//#include <depthmaps/depthmaputils.h>
#include "pcd_io.h"
#include "icp.h"
#include "basictypes/misc.h"
#include "heightmap.h"

using namespace std;

cv::Vec4f mat2quaternion(cv::Mat matrix){
    if(matrix.rows!=3 || matrix.cols!=3)
        throw runtime_error("The input must be a 3x3 matrix!");
    cv::Mat mat64;
    matrix.convertTo(mat64,CV_64FC1);
    cv::Matx33d mat(mat64);

    cv::Vec4f output;
    output[0]=std::copysign(std::sqrt(1+mat(0,0)-mat(1,1)-mat(2,2))/2.0,mat(2,1)-mat(1,2));
    output[1]=std::copysign(std::sqrt(1-mat(0,0)+mat(1,1)-mat(2,2))/2.0,mat(0,2)-mat(2,0));
    output[2]=std::copysign(std::sqrt(1-mat(0,0)-mat(1,1)+mat(2,2))/2.0,mat(1,0)-mat(0,1));
    output[3]=std::sqrt(1+mat(0,0)+mat(1,1)+mat(2,2))/2.0;

    return output;
}

PointCloud segment_subject(PointCloud &in_pc){
    PointCloud out_pc;
    for(size_t i=0;i<in_pc.size();i++){
        if(in_pc.points[i].z>0.15){
            out_pc.points.push_back(in_pc.points[i]);
            out_pc.colors.push_back(in_pc.colors[i]);
            out_pc.normals.push_back(in_pc.normals[i]);
        }
    }
    return out_pc;
}

vector<cv::Point3f> read_click_positions(string path){
    ifstream infile(path,ios_base::binary);
    if(!infile.is_open())
        throw runtime_error("Could not open the click file to read at:"+path);
    size_t num_points;
    infile.read((char*)&num_points,sizeof(size_t));
    vector<cv::Point3f> output(num_points);
    for(cv::Point3f &p:output){
        infile.read((char*)&p.x,sizeof p.x);
        infile.read((char*)&p.y,sizeof p.y);
        infile.read((char*)&p.z,sizeof p.z);
    }
    return output;
}

void draw_3d_line_points(cv::Point3f start, const cv::Point3f end, int steps, cv::Vec3b start_color, cv::Vec3b end_color, PointCloud& pc){
    cv::Point3d step=(end-start)/steps;
    cv::Vec3d color_step=(cv::Vec3d(end_color)-cv::Vec3d(start_color))/double(steps);
    cv::Point3d point=start;
    for(int i=0; i<steps; i++,point+=step) {
        pc.points.push_back(point);
        pc.colors.push_back(start_color+cv::Vec3b(i*color_step));
        pc.normals.emplace_back(0,0,0);
    }
}

void print_usage(){
    cout<<"Args: <dest_data_folder> <source_data_folder> [--with-icp]"<<endl;
}

int main(int argc, char* argv[]){
    if(argc<3){
        print_usage();
        return -1;
    }
    bool with_icp=false;
    if(argc>3){
        if(string(argv[3])=="--with-icp"){
            with_icp=true;
            cout<<"With ICP"<<endl;
        }
    }
    //read args
    string dest_folder=string(argv[1])+"/",source_folder=string(argv[2])+"/";

    string hm_file_name="merged.heightmap";
    //read heightmaps
    HeightMap src_hm,dst_hm;
    src_hm.readFromFile(source_folder+hm_file_name);
    dst_hm.readFromFile(dest_folder+hm_file_name);

    PointGrid src_pg=src_hm.getPointGrid();
    src_pg.calcNormals();
    PointGrid dst_pg=dst_hm.getPointGrid();
    dst_pg.calcNormals();

    PointCloud dest_pc=dst_pg.getPointCloud();
    PointCloud source_pc=src_pg.getPointCloud();
    //segment the relevant part
    dest_pc=segment_subject(dest_pc);
//    PCD_IO::write_pcd(ref_folder+"/segmented.pcd",ref_pc);
    source_pc=segment_subject(source_pc);

    //get click transfrom
    vector<cv::Point3f> click_positions_dest=read_click_positions(dest_folder+"/mousepoints");
    vector<cv::Point3f> click_positions_source=read_click_positions(source_folder+"/mousepoints");
    cv::Mat click_transfrom=ucoslam::rigidBodyTransformation_Horn1987(click_positions_source,click_positions_dest,true);

    PointCloud clicked_pc;
    for(size_t i=0;i<click_positions_source.size();i++){
        draw_3d_line_points(click_positions_source[i],click_positions_dest[i],100,cv::Vec3f(0,255,0),cv::Vec3f(255,0,0),clicked_pc);
        cout<<"from: "<<click_positions_source[i]<<" to "<<click_positions_dest[i]<<endl;
    }

    PCD_IO::write_pcd(source_folder+"/clicked_pointcloud.pcd",clicked_pc);

    source_pc=source_pc.transform(click_transfrom);//transfrom the source point cloud
    //refine it

    const int num_iterations=20;
    const pair<double,double> radius_interval(0.002,0.002);
    ICP icp(source_pc,dest_pc,0.002);
    cv::Mat T=icp.exec(radius_interval,num_iterations,true);
    cv::Mat final_T=click_transfrom;
    if(with_icp)
        final_T=T*final_T;
    cout<<"T: "<<T<<endl;
    cout<<"Final transform: "<<final_T<<endl;

    ICP::transformPointcloud(source_pc.points,T);
    PCD_IO::write_pcd(source_folder+"/transformed_source.pcd",source_pc);

    //write to file
    ofstream out_file(source_folder+"/transform2ref",ios_base::binary);
    if(!out_file.is_open())
        throw runtime_error("Could not open the output file to write at: "+source_folder+"/transform2ref");

    for(int i=0;i<3;i++)
        for(int j=0;j<4;j++)
            out_file.write((char*)&final_T.at<float>(i,j),sizeof(float));

    return 0;
}
