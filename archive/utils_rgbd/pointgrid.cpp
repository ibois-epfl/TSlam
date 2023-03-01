#include "pointgrid.h"
#include <opencv2/imgproc.hpp>
#include <limits>
#include <iostream>

using namespace std;

PointGrid::PointGrid(){
    T=cv::Mat::eye(4,4,CV_32FC1);
}

PointGrid::PointGrid(const cv::Mat pg, const cv::Mat transform, const cv::Mat c, const cv::Mat m)
{
    if(pg.type() != CV_32FC3)
        throw runtime_error("The point grid must be a cv::Mat of type CV_32FC3.");
    pg.copyTo(grid);

    if(!transform.empty()){
        if(transform.channels() != 1 || transform.rows != 4 || transform.cols !=4)
            throw runtime_error("The transformation must be a single channel 4x4 matrix.");
        transform.convertTo(T,CV_32FC1);
    }
    else{
        T=cv::Mat::eye(4,4,CV_32FC1);
    }

    if(!c.empty()){
        if(c.type() != CV_8UC3)
            throw runtime_error("The color image for point grid must be of type CV_8UC3.");
        c.copyTo(color);
    }

    if(!m.empty()){
        if(m.type() != CV_8UC1)
            throw runtime_error("The mask must have the type CV_8UC1.");
        m.copyTo(mask);
    }
    else
        mask=cv::Mat(grid.rows,grid.cols,CV_8UC1,255);//create the mask and fix the grid

    for(int r=0;r<pg.rows;r++){
        cv::Vec3f *grid_row=grid.ptr<cv::Vec3f>(r);
        uchar *mask_row=mask.ptr<uchar>(r);
        for(int c=0;c<pg.cols;c++)
            if(mask_row[c]!=255 || isnan(grid_row[c][0])){//invalid pixel
                grid_row[c]=cv::Vec3f(0,0,0);
                mask_row[c]=0;
            }
    }
}

cv::Mat PointGrid::getPlaneDists(std::pair<cv::Vec3f,float> plane_coeffs){
    cv::Mat grid_mat=getGridMat();

    cv::Mat plane_dists(grid_mat.rows,grid_mat.cols,CV_32FC1,cv::Scalar(std::numeric_limits<float>::max()));

    double coeffs_norm=cv::norm(plane_coeffs.first);
    for(int r=0;r<plane_dists.rows;r++){
        float *dists_row=plane_dists.ptr<float>(r);
        cv::Vec3f *grid_row=grid_mat.ptr<cv::Vec3f>(r);
        uchar *mask_row=mask.ptr<uchar>(r);
        for(int c=0;c<plane_dists.cols;c++)
            if(mask_row[c])
                dists_row[c]=(grid_row[c].dot(plane_coeffs.first)+plane_coeffs.second)/coeffs_norm;
    }
    return plane_dists;
}

cv::Mat PointGrid::getGridMat(){
    cv::Mat output;
    grid.copyTo(output);

    cv::Matx33f R(T(cv::Range(0,3),cv::Range(0,3)));
    cv::Vec3f t(T(cv::Range(0,3),cv::Range(3,4)));

    for(int r=0;r<output.rows;r++){
        cv::Vec3f *output_row=output.ptr<cv::Vec3f>(r);
        uchar* mask_row=mask.ptr<uchar>(r);
        for(int c=0;c<output.cols;c++)
            if(mask_row[c])
                output_row[c]=R*output_row[c]+t;
            else
                output_row[c]=cv::Vec3f(NAN,NAN,NAN);
    }

    return output;
}

PointCloud PointGrid::getPointCloud(){
    PointCloud tmp;
    for(int r=0;r<mask.rows;r++){
        uchar *mask_row=mask.ptr<uchar>(r);
        cv::Vec3f *grid_row=grid.ptr<cv::Vec3f>(r);
        cv::Vec3b *color_row;
        cv::Vec3f *normals_row;
        if(!color.empty())
            color_row=color.ptr<cv::Vec3b>(r);
        if(!normals.empty())
            normals_row=normals.ptr<cv::Vec3f>(r);
        for(int c=0;c<mask.cols;c++)
            if(mask_row[c]==255){
                tmp.points.push_back(grid_row[c]);
                if(!color.empty())
                    tmp.colors.push_back(color_row[c]);
                if(!normals.empty())
                    tmp.normals.push_back(normals_row[c]);
            }
    }
    if(T.empty())
        return tmp;
    else
        return tmp.transform(T);
}

void PointGrid::calcNormals(){
    normals=cv::Mat::zeros(grid.rows,grid.cols,CV_32FC3);

    vector<cv::Vec3f*> fast_grid(grid.rows);
    vector<uchar*> fast_mask(mask.rows);
    for(int r=0;r<grid.rows;r++){
        fast_grid[r]=grid.ptr<cv::Vec3f>(r);
        fast_mask[r]=mask.ptr<uchar>(r);
    }
    for(int y=1;y<grid.rows-1;y++){
        uchar *mask_row=mask.ptr<uchar>(y);

        cv::Vec3f *normals_row=normals.ptr<cv::Vec3f>(y);
        for(int x=1;x<grid.cols-1;x++){
            if(mask_row[x]==255){//if a valid pixel to have normals
                cv::Vec3f neighbours[8];
                bool neighbour_mask[8];
                for(int i=0;i<8;i++){
                    int j=(i+2)%8;
                    int dx=0,dy=0;
                    if(i%4)
                        dx=(i/4)*2-1;
                    if(j%4)
                        dy=(j/4)*2-1;
                    neighbours[i]=fast_grid[y+dy][x+dx];
                    neighbour_mask[i]=fast_mask[y+dy][x+dx];
                }

                cv::Vec3f point=fast_grid[y][x],av_normal=cv::Vec3f(0,0,0);
                int num_normals=0;
                for(int i=0;i<8;i++)
                    if(neighbour_mask[i] && neighbour_mask[(i+1)%8]){
                        av_normal+=cv::normalize((neighbours[i]-point).cross(neighbours[(i+1)%8]-point));
                        num_normals++;
                    }

                if(num_normals>0)
                    normals_row[x]=cv::normalize(av_normal/num_normals);
            }
        }
    }
}

cv::Mat PointGrid::getNormalsMat(){
    return normals.clone();
}

void PointGrid::setNormalsMat(const cv::Mat &input){
    input.copyTo(normals);
}

PointGrid PointGrid::getSmoothGrid(int radius, bool blur_color){
    cv::Size filter_size(radius,radius);
    cv::Mat out_grid,out_color;

    cv::Mat blurred_map_mask,blurred_map;
    vector<cv::Mat> dims;
    cv::split(grid,dims);
    cv::Mat depth_map=dims[2];//smoothing is done only on the Z values
    mask.convertTo(blurred_map_mask,CV_32FC1);
    blurred_map_mask /= 255.0;
    cv::GaussianBlur(blurred_map_mask,blurred_map_mask,filter_size,0);
    cv::GaussianBlur(depth_map,blurred_map,filter_size,0);

    dims[2]=blurred_map/blurred_map_mask;
    cv::merge(dims,out_grid);

    if(blur_color){
        cv::Mat blurred_color,blurred_color_mask;
        color.convertTo(blurred_color,CV_32FC3);
        cv::GaussianBlur(blurred_color,blurred_color,filter_size,0);
        vector<cv::Mat> bcm_channels(3,blurred_map_mask);
        cv::merge(bcm_channels,blurred_color_mask);
        blurred_color/=blurred_color_mask;
        blurred_color.convertTo(out_color,CV_8UC3);
    }
    else
        color.copyTo(out_color);

    return PointGrid(out_grid,T,out_color,mask);
}
