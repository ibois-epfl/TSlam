#include "pcd_io.h"
#include <iostream>
#include <fstream>

using namespace std;

PCD_IO::PCD_IO()
{

}

void PCD_IO::read_pcd(string path, vector<cv::Point3f> &points, vector<cv::Vec3b> *colors, vector<cv::Vec3f> *normals) {
    ifstream in_file(path,ios_base::binary);
    if(!in_file.is_open())
        throw std::runtime_error("Could not open a file to read at: "+path);

    size_t num_points;
    char buffer[100];
    bool with_color=false;
    bool with_normals=false;
    while(true){
        in_file.getline(buffer,100);
        string line(buffer);
        istringstream ss(line);
        string arg;
        ss>>arg;
        if(arg=="FIELDS"){
            if(line!="FIELDS x y z"){
                if(line=="FIELDS x y z rgb")
                    with_color=true;
                else if(line=="FIELDS x y z normal_x normal_y normal_z"){
                    with_normals=true;
                }
                else if(line=="FIELDS x y z normal_x normal_y normal_z rgb"){
                    with_color=true;
                    with_normals=true;
                }
                else
                    throw runtime_error("The FIELDS are not supported they should be: x y z [normal_x normal_y normal_z] [rgb]");
            }
        }
        else if(arg=="SIZE"){
            int bytes;
            while(ss>>bytes)
                if(bytes!=4)
                    throw runtime_error("They SIZE arguments are not supported they should be all 4.");
        }
        else if(arg=="POINTS"){
            ss>>num_points;
            points.resize(num_points);
        }
        else if(arg=="DATA"){
            if(line != "DATA binary")
                throw runtime_error("Non binary DATA is not supported.");
            break;
        }
    }

    if(with_color && colors!=NULL)
        colors->resize(num_points);
    if(with_normals && normals!=NULL)
        normals->resize(num_points);

    //read the points in binary form
    for(size_t i=0; i<points.size(); i++) {
        cv::Vec3f point;
        for(int j=0; j<3; j++)
            in_file.read((char*)(&point[j]),sizeof point[j]);
        points[i]=point;
        if(with_normals){
            cv::Vec3f normal;
            for(int j=0; j<3; j++)
                in_file.read((char*)(&normal[j]),sizeof normal[j]);
            if(normals!=NULL)
                (*normals)[i]=normal;
        }

        if(with_color){
            char32_t color=0;
            in_file.read((char*)(&color),sizeof color);
            if(colors!=NULL)
                for(int j=2; j>=0; j--) {
                    (*colors)[i][j]=color%256;
                    color >>= 8;
                }
        }
    }
    cout<<points.size()<<" points were read."<<endl;
}

void PCD_IO::write_ply(std::string path, const PointCloud &pc){
    ofstream of(path,ios_base::binary);
    if(!of.is_open())
        throw std::runtime_error("Could not open a file to write at: "+path);
    of<<"ply"<<endl;
    of<<"format binary_little_endian 1.0"<<endl;
    of<<"element vertex "<<pc.points.size()<<endl;
    of<<"property float x"<<endl;
    of<<"property float y"<<endl;
    of<<"property float z"<<endl;
    if(!pc.colors.empty()){
        of<<"property uchar blue"<<endl;
        of<<"property uchar green"<<endl;
        of<<"property uchar red"<<endl;
    }
    if(!pc.normals.empty()){
        of<<"property float nx"<<endl;
        of<<"property float ny"<<endl;
        of<<"property float nz"<<endl;
    }
    of<<"end_header"<<endl;
    for(size_t i=0;i<pc.points.size();i++){
        const cv::Point3f &point=pc.points[i];
        of.write((char*)&point.x, sizeof point.x);
        of.write((char*)&point.y, sizeof point.y);
        of.write((char*)&point.z, sizeof point.z);
        if(!pc.colors.empty()){
            const cv::Vec3b &color=pc.colors[i];
            for(int j=0;j<3;j++)
                of.write((char*)&color[j], sizeof color[j]);
        }
        if(!pc.normals.empty()){
            const cv::Vec3f &normal=pc.normals[i];
            for(int j=0;j<3;j++)
                of.write((char*)&normal[j], sizeof normal[j]);
        }
    }
}

void PCD_IO::write_pcd(string path, const vector<cv::Point3f> &points, const vector<cv::Vec3b> &colors, const vector<cv::Vec3f> &normals) {
    ofstream of(path,ios_base::binary);
    if(!of.is_open())
        throw std::runtime_error("Could not open a file to write at: "+path);
    cout<<points.size()<<" points to be written."<<endl;
    of<<"# .PCD v0.7 - Point Cloud Data File Format"<<endl;
    of<<"VERSION 0.7"<<endl;

    string fields_line="FIELDS x y z",size_line="SIZE 4 4 4",type_line="TYPE F F F",count_line="COUNT 1 1 1";

    if(!normals.empty()){
        fields_line+=" normal_x normal_y normal_z";
        size_line+=" 4 4 4";
        type_line+=" F F F";
        count_line+=" 1 1 1";
    }
    if(!colors.empty()){
        fields_line+=" rgb";
        size_line+=" 4";
        type_line+=" F";
        count_line+=" 1";
    }

    of<<fields_line<<endl;
    of<<size_line<<endl;
    of<<type_line<<endl;
    of<<count_line<<endl;

    of<<"WIDTH "<<points.size()<<endl;
    of<<"HEIGHT 1"<<endl;
    of<<"VIEWPOINT 0 0 0 1 0 0 0"<<endl;
    of<<"POINTS "<<points.size()<<endl;
    of<<"DATA binary"<<endl;
    //write the points in binary form
    for(size_t i=0; i<points.size(); i++) {
        cv::Vec3f point(points[i]);
        for(int j=0; j<3; j++) {
            float value=point[j];
            of.write((char*)(&value),sizeof value);
        }
        if(!normals.empty())
            for(int j=0; j<3; j++)
                of.write((char*)(&normals[i][j]),sizeof(float));
        if(!colors.empty()){
            char32_t color=0;
            for(int j=0; j<3; j++) {
                color <<= 8;
                color += char32_t(colors[i][j]);
            }
            of.write((char*)(&color),sizeof color);
        }
    }
}

void PCD_IO::write_pcd(string path, cv::Mat &xyz, cv::Mat &color_image) {
    ofstream of(path,ios_base::binary);
    if(!of.is_open())
        throw std::runtime_error("Could not open a file to write at: "+path);
    cout<<xyz.cols*xyz.rows<<" points to be written."<<endl;
    of<<"# .PCD v0.7 - Point Cloud Data File Format"<<endl;
    of<<"VERSION 0.7"<<endl;
    of<<"FIELDS x y z rgb"<<endl;
    of<<"SIZE 4 4 4 4"<<endl;
    of<<"TYPE F F F F"<<endl;
    of<<"COUNT 1 1 1 1"<<endl;
    of<<"WIDTH "<<xyz.cols<<endl;
    of<<"HEIGHT "<<xyz.rows<<endl;
    of<<"VIEWPOINT 0 0 0 1 0 0 0"<<endl;
    of<<"POINTS "<<xyz.cols*xyz.rows<<endl;
    of<<"DATA binary"<<endl;
    //write the points in binary form
    for(size_t r=0; r<xyz.rows; r++){
        cv::Vec3f *xyz_row=xyz.ptr<cv::Vec3f>(r);
        cv::Vec3b *color_row=color_image.ptr<cv::Vec3b>(r);
        for(size_t c=0; c<xyz.cols; c++){
            cv::Vec3f point=xyz_row[c];
            cv::Vec3b colors=color_row[c];
            char valid=1;
            if(point[0]==0 && point[1]==0 && point[2]==0){
                point[0]=point[1]=point[2]=NAN;
                valid=0;
            }

            for(int i=0; i<3; i++) {
                float value=point[i];
                of.write((char*)(&value),sizeof value);
            }
            char32_t color=0;
            for(int i=0; i<3; i++) {
                color <<= 8;
                color += char32_t(colors[i]);
            }
            color <<= 8;
            color += char32_t(valid);

            if(std::isnan(point[0]))
                of.write((char*)(&point[0]),sizeof point[0]);
            else
                of.write((char*)(&color),sizeof color);
        }
    }
}

void PCD_IO::write_pcd(string path, const PointCloud &pc){
    write_pcd(path, pc.points, pc.colors, pc.normals);
}

PointCloud PCD_IO::read_pcd(string path){
    PointCloud pc;
    read_pcd(path,pc.points,&pc.colors,&pc.normals);
    return pc;
}
