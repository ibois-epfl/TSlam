/**
This file is part of TSLAM and distributed under the terms of the GPLv3.

The modified version: Copyright (C) 2023 Andrea Settimi <andrea dot settimi at epfl dot ch>, Hong-Bin Yang <hong dash bin dot yang at epfl dot ch> (IBOIS, EPFL)
The original project: Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas, MODIFIED BY Andrea Settimi and Hong-Bin Yang ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
*/
#include "map_types/marker.h"
#include "basictypes/misc.h"
#include "basictypes/hash.h"
#include "basictypes/io_utils.h"
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
namespace tslam{



std::vector<cv::Point3f> Marker::get3DPoints(bool global_ref)const{
    vector<cv::Point3f> marker_points = points3d;
    cv::Mat m44=pose_g2m;
    if (global_ref){
        for(auto &p:marker_points)
            p=mult<float>(m44,p);
    }
    return marker_points;
}

void Marker::toStream(std::ostream &str)const {
    str.write((char*)&id,sizeof(id));
    toStream__(points3d,str);
    pose_g2m.toStream(str);
    toStream__(frames,str);
    toStream__(dict_info,str);

}
void Marker::fromStream(std::istream &str) {
    str.read((char*)&id,sizeof(id));
    fromStream__(points3d,str);
    pose_g2m.fromStream(str);
    fromStream__(frames,str);
    fromStream__(dict_info,str);

}

//std::vector<cv::Point3f> Marker::get3DPointsLocalRefSystem( float size ){
// return { cv::Point3f ( -size/2., size/2.,0 ),cv::Point3f ( size/2., size /2.,0 ),
//                                          cv::Point3f ( size/2., -size/2.,0 ),cv::Point3f ( -size/2., -size/2.,0 )  };

//}


//vector<cv::Point3f> Marker::get3DPoints(Se3Transform m44,float size,bool global_ref){
//    vector<cv::Point3f> marker_points = { cv::Point3f ( -size/2., size/2.,0 ),cv::Point3f ( size/2., size /2.,0 ),
//                                          cv::Point3f ( size/2., -size/2.,0 ),cv::Point3f ( -size/2., -size/2.,0 )  };
//    if (global_ref){
//        for(auto &p:marker_points)
//            p=m44*p;
//    }
//    return marker_points;
//}
uint64_t Marker::getSignature(bool print) const{
    print=false;
    Hash sig;
    sig+=id;
    if(print)cout<<"\t\t\t1. marker"<<id<<" :"<<sig<<endl;
    if(print)cout<<"\t\t\t1. marker"<<id<<" pose=\n\t\t\t\t"<<pose_g2m<<endl;
    if(pose_g2m.isValid()){
        for(int i=0;i<16;i++)
            sig+=pose_g2m.ptr<float>(0)[i];
    }
    if(print)cout<<"\t\t\t2. marker"<<id<<" :"<<sig<<endl;
    sig.add(frames.begin(),frames.end());
    if(print)cout<<"\t\t\t3. marker"<<id<<" :"<<sig<<endl;
    sig+=dict_info;
    if(print)cout<<"\t\t\t4. marker"<<id<<" :"<<sig<<endl;
    return sig;
}

void MarkerObservation::toStream(std::ostream &str)const{
    Marker::toStream(str);
    toStream__(corners,str);
    toStream__(und_corners,str);
    poses.toStream(str);
}



void MarkerObservation::fromStream(std::istream &str){
    Marker::fromStream(str);
    fromStream__(corners,str);
    fromStream__(und_corners,str);
    poses.fromStream(str);
}

uint64_t MarkerObservation::getSignature()const{
    Hash sig;
    for(const auto &c:corners){
        sig+=c.x;
        sig+=c.y;
    }
    for(const auto &c:und_corners){
        sig+=c.x;
        sig+=c.y;
    }
    sig+=id;
    sig+=poses.getSignature();
    sig+=dict_info;
    return sig;
}


void MarkerObservation::draw(cv::Mat& in,  cv::Scalar color, int lineWidth, bool writeId, bool writeInfo) const
{

    auto _to_string=[](int i){
        std::stringstream str;str<<i;return str.str();
        };

    if (corners.size() != 4)
        return;
    if (lineWidth == -1)  // auto
        lineWidth = static_cast<int>(std::max(1.f, float(in.cols) / 1000.f));
    cv::line(in, corners[0], corners[1], color, lineWidth);
    cv::line(in, corners[1], corners[2], color, lineWidth);
    cv::line(in, corners[2], corners[3], color, lineWidth);
    cv::line(in, corners[3], corners[0], color, lineWidth);

    auto p2 =  cv::Point2f(2.f * static_cast<float>(lineWidth), 2.f * static_cast<float>(lineWidth));
    cv::rectangle(in, corners[0] - p2, corners[0] + p2, cv::Scalar(0, 0, 255, 255), -1  );
    cv::rectangle(in, corners[1] - p2, corners[1] + p2, cv::Scalar(0, 255, 0, 255), lineWidth );
    cv::rectangle(in, corners[2] - p2, corners[2] + p2, cv::Scalar(255, 0, 0, 255), lineWidth);



    if (writeId)
    {
        // determine the centroid
        cv::Point cent(0, 0);
        for (int i = 0; i < 4; i++)
        {
            cent.x += static_cast<int>(corners[i].x);
            cent.y +=  static_cast<int>(corners[i].y);
        }
        cent.x /= 4;
        cent.y /= 4;
        std::string str;
        if(writeInfo) str+= dict_info +":";
        if(writeId)str+=_to_string(id);
        cv::putText(in,str, cent,  cv::FONT_HERSHEY_SIMPLEX, std::max(0.5f, float(lineWidth) * 0.3f),
                    cv::Scalar(255 - color[0], 255 - color[1], 255 - color[2], 255), std::max(lineWidth, 2));
    }
}
}
