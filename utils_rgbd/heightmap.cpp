#include "heightmap.h"
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<fstream>
#include<iostream>

using namespace std;

HeightMap::HeightMap()
{

}

void HeightMap::init_mats(int rows, int cols){
    height_map.release();
    map_color.release();
    mask.release();
    height_map=cv::Mat::zeros(rows,cols,CV_32FC1);
    map_color=cv::Mat::zeros(rows,cols,CV_8UC3);
    mask=cv::Mat::zeros(rows,cols,CV_8UC1);
}

cv::Mat HeightMap::getRGB() const{
    return map_color.t();
}

cv::Mat HeightMap::getHeightImage() const{
    double min,max;
    int min_i,max_i;
    cv::minMaxIdx(height_map,&min,&max,&min_i,&max_i,mask);
    min=0;
    cv::Mat output=(cv::max(min,height_map)-min)/(max-min);
    cv::Mat float_mask;
    mask.convertTo(float_mask,CV_32FC1);
    output=float_mask.mul(output);

//    cv::Mat zero_mask(output<=0);
//    zero_mask.convertTo(zero_mask,CV_32FC1);
//    output+=zero_mask;

//    vector<cv::Mat> mat_arr(3);
//    mat_arr[1]=1-mat_arr[2];
//    mat_arr[0]=cv::Mat::zeros(mat_arr[2].rows,mat_arr[2].cols,mat_arr[2].type());
//    cv::Mat output;
//    cv::merge(mat_arr,output);
    return output.t();
}

HeightMap HeightMap::getDiffFrom(const HeightMap &ref, float max_abs_diff){
    if(ref.height_map.rows != height_map.rows || ref.height_map.cols != height_map.cols){
        throw runtime_error("The dimensions of the reference heightmap do not match for comparison: "+
                            to_string(height_map.rows)+"!="+to_string(ref.height_map.rows)+" OR"+
                            to_string(height_map.cols)+"!="+to_string(ref.height_map.cols));
    }
    HeightMap output_map = *this;
    output_map.mask = mask & ref.mask;
    cv::subtract(height_map,ref.height_map,output_map.height_map,output_map.mask);
    output_map.map_color=cv::Mat::zeros(height_map.rows,height_map.cols,CV_8UC3);
    double min,max;
    cv::minMaxIdx(output_map.height_map,&min,&max);
//    max_abs_diff=std::max(std::abs(min),std::abs(max));
    cout<<"min:"<<min<<endl<<"max:"<<max<<endl;
    cout<<"max_abs_diff:"<<max_abs_diff<<endl;
    for(int r=0;r<height_map.rows;r++){
        const uchar *out_mask_row=output_map.mask.ptr<uchar>(r);
        const float *out_hm_row=output_map.height_map.ptr<float>(r);
        cv::Vec3b *out_color_row=output_map.map_color.ptr<cv::Vec3b>(r);
        for(int c=0;c<height_map.cols;c++)
            if(out_mask_row[c]){
                float abs_diff=std::abs(out_hm_row[c]);
                cv::Vec3b &color=out_color_row[c];
                color[2]=std::min(abs_diff/max_abs_diff,1.0f)*255;
                color[0]=255-color[2];
            }
    }
    return output_map;
}

HeightMap HeightMap::getBlurredMap(cv::Size filter_size, bool blur_color){
    HeightMap out_hm;
    out_hm.grid_step=grid_step;
    out_hm.ranges=ranges;
    out_hm.surface_thickness=surface_thickness;
    cv::Mat blurred_map_mask,blurred_color_mask,blurred_map;
    mask.convertTo(blurred_map_mask,CV_32FC1);
    blurred_map_mask /= 255.0;
    cv::GaussianBlur(blurred_map_mask,blurred_map_mask,filter_size,0);
    cv::GaussianBlur(height_map,blurred_map,filter_size,0);
    out_hm.height_map=blurred_map/blurred_map_mask;
    mask.copyTo(out_hm.mask);
    if(blur_color){
        cv::Mat blurred_color;
        map_color.convertTo(blurred_color,CV_32FC3);
        cv::GaussianBlur(blurred_color,blurred_color,filter_size,0);
        vector<cv::Mat> bcm_channels(3,blurred_map_mask);
        cv::merge(bcm_channels,blurred_color_mask);
        blurred_color/=blurred_color_mask;
        blurred_color.convertTo(out_hm.map_color,CV_8UC3);
    }
    else
        map_color.copyTo(out_hm.map_color);
    return out_hm;
}

PointCloud HeightMap::getPointcloud(){
    PointCloud out_pc;
    for(size_t r=0;r<height_map.rows;r++){
        float *height_map_row=height_map.ptr<float>(r);
        uchar *map_mask_row=mask.ptr<uchar>(r);
        cv::Vec3b *map_image_row=map_color.ptr<cv::Vec3b>(r);
        float row=r+0.5;
        for(size_t c=0;c<height_map.cols;c++){
            float col=c+0.5;
            if(map_mask_row[c]!=0){
                out_pc.points.push_back(cv::Point3f(ranges.x+col*grid_step,ranges.y+row*grid_step,height_map_row[c]));
                const cv::Vec3b& color=map_image_row[c];
                out_pc.colors.push_back(cv::Vec3b(color[2],color[1],color[0]));
            }
        }
    }
    return out_pc;
}

PointGrid HeightMap::getPointGrid() const{
    cv::Mat grid(height_map.rows,height_map.cols,CV_32FC3);

    for(int r=0;r<grid.rows;r++){
        cv::Vec3f *grid_row=grid.ptr<cv::Vec3f>(r);
        const float *hm_row=height_map.ptr<float>(r);
        const uchar *map_mask_row=mask.ptr<uchar>(r);
        float y=r+0.5;
        for(int c=0;c<grid.cols;c++)
            if(map_mask_row[c]){
                float x=c+0.5;
                grid_row[c][0]=x*grid_step+ranges.x;
                grid_row[c][1]=y*grid_step+ranges.y;
                grid_row[c][2]=hm_row[c];
            }
    }

    return PointGrid(grid,cv::Mat(),map_color,mask);
}

HeightMap HeightMap::merge(const std::vector<HeightMap> &in_hms){
    size_t num_hms=in_hms.size();
    if(num_hms==0)
        throw runtime_error("The input HeightMap array to the merge function is empty!");

    int rows=in_hms[0].height_map.rows;
    int cols=in_hms[0].height_map.cols;

    for(size_t i=1;i<in_hms.size();i++)//check the sizes
        if(in_hms[i].height_map.rows!=rows || in_hms[i].height_map.cols!=cols)
            throw runtime_error("The input HeightMaps to the merge function must have the same size!");

    //merge
    HeightMap out_hm=in_hms[0];
    out_hm.init_mats(rows,cols);

    vector<const float*> hms_row(num_hms);
    vector<const cv::Vec3b*> hms_color_row(num_hms);
    vector<const uchar*> hms_mask_row(num_hms);
    for(int r=0;r<rows;r++){
        //get the fast access rows from inputs
        for(size_t i=0;i<num_hms;i++){
            hms_row[i]=in_hms[i].height_map.ptr<float>(r);
            hms_color_row[i]=in_hms[i].map_color.ptr<cv::Vec3b>(r);
            hms_mask_row[i]=in_hms[i].mask.ptr<uchar>(r);
        }
        //get the output rows
        float *out_hm_row=out_hm.height_map.ptr<float>(r);
        cv::Vec3b *out_color_row=out_hm.map_color.ptr<cv::Vec3b>(r);
        uchar *out_mask_row=out_hm.mask.ptr<uchar>(r);
        //iterate over cells
        for(int c=0;c<cols;c++){
            size_t num_valid_cells=0;
            float sum_heights=0;
            cv::Vec3f sum_colors(0,0,0);
            bool valid_out_cell=false;
            //go through the height maps
            for(size_t i=0;i<num_hms;i++){
                if(hms_mask_row[i][c]!=0){//if the cell of that height map is valid
                    valid_out_cell=true;
                    num_valid_cells++;
                    sum_heights+=hms_row[i][c];
                    sum_colors+=hms_color_row[i][c];
                }
            }
            if(valid_out_cell){
                out_mask_row[c]=255;
                out_hm_row[c]=sum_heights/num_valid_cells;
                out_color_row[c]=sum_colors/float(num_valid_cells);
            }
        }
    }
    return out_hm;
}

vector<HeightMap> HeightMap::fromPointClouds(const vector<PointCloud> &pc, const cv::Rect2f &in_ranges, float in_grid_step, float in_surface_thickness){
    vector<HeightMap> output(pc.size());
    for(size_t i=0;i<output.size();i++)
        output[i].fromPointCloud(pc[i],in_ranges,in_grid_step,in_surface_thickness);
    return output;
}

void HeightMap::fromPointCloud(const PointCloud &pc, const cv::Rect2f &in_ranges, float in_grid_step, float in_surface_thickness){
    ranges=in_ranges;
    grid_step=in_grid_step;
    surface_thickness=in_surface_thickness;

    vector<vector<vector<size_t>>> grid;
    unsigned int x_dim_size=(ranges.width-ranges.x)/grid_step;
    unsigned int y_dim_size=(ranges.height-ranges.y)/grid_step;
    //fill the grid
    grid.resize(y_dim_size,vector<vector<size_t>>(x_dim_size));
    for(int p=0;p<pc.points.size();p++){
        cv::Point3f point=pc.points[p];
        //get grid coordinates and check the ranges
        int grid_x=(point.x-ranges.x)/grid_step;
        if(grid_x<0 || grid_x>=x_dim_size)
            continue;
        int grid_y=(point.y-ranges.y)/grid_step;
        if(grid_y<0 || grid_y>=y_dim_size)
            continue;
        //insert the point into the grid
        grid[grid_y][grid_x].push_back(p);
    }
    //convert the grid to height map
    init_mats(y_dim_size,x_dim_size);
    for(size_t r=0;r<y_dim_size;r++){
        vector<vector<size_t>> &grid_row=grid[r];
        float *height_map_row=height_map.ptr<float>(r);
        uchar *map_mask_row=mask.ptr<uchar>(r);
        cv::Vec3b *map_image_row=map_color.ptr<cv::Vec3b>(r);
        for(size_t c=0;c<x_dim_size;c++){
            vector<size_t> &bin=grid_row[c];
            if(!bin.empty()){//put the average height of the points that are closer to the top point than the max_thickness
                size_t max_index=bin[0];
                float max_height=pc.points[max_index].z;
                for(size_t index:bin){
                    float height=pc.points[index].z;
                    if(height>max_height){
                        max_height=height;
                        max_index=index;
                    }
                }
                float sum_heights = 0;
                cv::Vec3i sum_colors(0,0,0);
                unsigned int num_points=0;
                for(size_t index:bin){
                    float height=pc.points[index].z;
                    if(height>=max_height-in_surface_thickness){
                        sum_heights+=height;
                        sum_colors+=pc.colors[index];
                        num_points++;
                    }
                }
                height_map_row[c]=sum_heights/float(num_points);
                map_image_row[c]=cv::Vec3i(sum_colors[2],sum_colors[1],sum_colors[0])/float(num_points);
                map_mask_row[c]=255;
            }
        }
    }
}

bool HeightMap::get3DCoords(cv::Point input,cv::Point3f &output){
    int row=input.y;
    int col=input.x;
    if(row>=mask.rows || row<0 || col>=mask.cols || col<0)
        return false;
    if(mask.at<uchar>(row,col)==0)
        return false;

    output.x=col*grid_step+ranges.x;
    output.y=row*grid_step+ranges.y;
    output.z=height_map.at<float>(row,col);
    return true;
}

cv::Point3f HeightMap::get3DCoords(cv::Point2f input){
    cv::Point3f output(NAN,NAN,NAN);
    input.x=std::max(0.0f,std::min(float(height_map.cols-1),input.x));
    input.y=std::max(0.0f,std::min(float(height_map.rows-1),input.y));
    int row=input.y+.5;
    int col=input.x+.5;
    if(mask.at<uchar>(row,col)){
        output.x=input.x*grid_step+ranges.x;
        output.y=input.y*grid_step+ranges.y;
        output.z=height_map.at<float>(row,col);
    }
    else{
        int radius=5;
        cv::Range x_range(std::max(0,col-radius),std::min(height_map.cols-1,col+radius));
        cv::Range y_range(std::max(0,row-radius),std::min(height_map.rows-1,row+radius));
        float min_dist=radius*2;
        float min_dist_val=NAN;
        for(int x=x_range.start;x<=x_range.end;x++)
            for(int y=y_range.start;y<=y_range.end;y++){
                if(mask.at<uchar>(y,x)){
                    float dy=input.y-y;
                    float dx=input.x-x;
                    float dist=std::sqrt(dx*dx+dy*dy);
                    if(dist<min_dist){
                        min_dist=dist;
                        min_dist_val=height_map.at<float>(y,x);
                    }
                }
            }
        if(!std::isnan(min_dist_val)){
            output.x=input.x*grid_step+ranges.x;
            output.y=input.y*grid_step+ranges.y;
            output.z=min_dist_val;
        }
    }
    return output;
}

void HeightMap::writeToFile(string path){
    ofstream out_file(path,ios_base::binary);
    if(!out_file.is_open())
        throw runtime_error("Cannot open a file to write the height map at: "+path);
    out_file.write((char*)&grid_step, sizeof grid_step);
    out_file.write((char*)&ranges.x, sizeof ranges.x);
    out_file.write((char*)&ranges.y, sizeof ranges.y);
    out_file.write((char*)&ranges.width, sizeof ranges.width);
    out_file.write((char*)&ranges.height, sizeof ranges.height);
    out_file.write((char*)&surface_thickness, sizeof surface_thickness);
    out_file.write((char*)&height_map.rows, sizeof height_map.rows);
    out_file.write((char*)&height_map.cols, sizeof height_map.cols);
    for(int r=0;r<height_map.rows;r++){
        const uchar* mask_row=mask.ptr<uchar>(r);
        const float* height_row=height_map.ptr<float>(r);
        const cv::Vec3b* color_row=map_color.ptr<cv::Vec3b>(r);
        for(int c=0;c<height_map.cols;c++){
            const uchar &mask_val=mask_row[c];
            out_file.write((char*)&mask_val,sizeof(uchar));
            if(mask_val!=0){
                out_file.write((char*)&height_row[c],sizeof(float));
                const cv::Vec3b &color=color_row[c];
                for(int i=0;i<3;i++)
                    out_file.write((char*)&color[i],sizeof(uchar));
            }
        }
    }
}

void HeightMap::readFromFile(string path){
    ifstream in_file(path,ios_base::binary);
    if(!in_file.is_open())
        throw runtime_error("Cannot open the height map file to read at: "+path);
    in_file.read((char*)&grid_step, sizeof grid_step);
    in_file.read((char*)&ranges.x, sizeof ranges.x);
    in_file.read((char*)&ranges.y, sizeof ranges.y);
    in_file.read((char*)&ranges.width, sizeof ranges.width);
    in_file.read((char*)&ranges.height, sizeof ranges.height);
    in_file.read((char*)&surface_thickness, sizeof surface_thickness);
    int rows,cols;
    in_file.read((char*)&rows, sizeof rows);
    in_file.read((char*)&cols, sizeof cols);

    init_mats(rows,cols);

    for(int r=0;r<rows;r++){
        uchar* mask_row=mask.ptr<uchar>(r);
        float* height_row=height_map.ptr<float>(r);
        cv::Vec3b* color_row=map_color.ptr<cv::Vec3b>(r);
        for(int c=0;c<cols;c++){
            uchar &mask_val=mask_row[c];
            in_file.read((char*)&mask_val,sizeof(uchar));
            if(mask_val!=0){
                in_file.read((char*)&height_row[c],sizeof(float));
                const cv::Vec3b &color=color_row[c];
                for(int i=0;i<3;i++)
                    in_file.read((char*)&color[i],sizeof(uchar));
            }
        }
    }
}
