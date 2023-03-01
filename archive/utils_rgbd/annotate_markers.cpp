#include <iostream>
#include "heightmap.h"
using namespace std;

void print_usage(){
    cout<<"Args: <path_to_heightmap_folder>"<<endl;
}

int main(int argc, char* argv[]){
    if(argc<2){
        print_usage();
        return -1;
    }
    HeightMap hm;
    hm.readFromFile(argv[1]);

    hm.height_map;

    cv::setMouseCallback("new_color",call_back_mouse,&new_mouse_coords);

    while(new_mouse_coords.size()<9){
        int prev_size=new_mouse_coords.size();
        new_hm_color.copyTo(new_hm_color_show);
        for(cv::Point p:new_mouse_coords)
            cv::drawMarker(new_hm_color_show,p,cv::Scalar(0,0,255));
        cv::imshow("new_color",new_hm_color_show);
        cv::waitKey(10);
        if(prev_size<new_mouse_coords.size()){
            cv::Point3f point_3d;
            if(new_height_map.get3DCoords(new_mouse_coords.back(),point_3d))
                new_mouse_3D_coords.push_back(point_3d);
            else
                new_mouse_coords.pop_back();
        }
        else if(prev_size>new_mouse_coords.size())
            new_mouse_3D_coords.pop_back();
    }
    new_mouse_coords.pop_back();
    new_mouse_3D_coords.pop_back();
    cout<<"new 3D coords"<<endl;
}
