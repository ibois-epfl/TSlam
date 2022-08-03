/*
 * BSD 2-Clause License

Copyright (c) 2018, Hamid Sarmadi, Rafael Muñoz Salinas
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef SEQUENCEOPTIMIZER_H
#define SEQUENCEOPTIMIZER_H

#include "map.h"
#include "nanogo.h"

class SequenceOptimizer
{
    class Frame2Global: public nanogo::Variable<double>{
    public:
        tslam::Se3Transform _G2F;
        int32_t id=-1;
        int32_t extra=8347947;

        void setParams(tslam::Se3Transform G2F,int Id){
            needPreparation=true;
            _G2F=G2F;
            id=Id;
            cv::Mat r=cv::Mat::zeros(3,1,CV_32FC1);//G2F.getRvec();
            cv::Mat t=cv::Mat::zeros(3,1,CV_32FC1);//G2F.getTvec();
            this->resize(6);
            for(int i=0;i<3;i++)
                (*this)[i]=r.at<float>(i);
            for(int i=3;i<6;i++)
                (*this)[i]=t.at<float>(i-3);

        }
        inline void prepare(){
            //converting from the vector representation to matrix
            cv::Mat r(3,1,CV_32FC1);
            for(int i=0;i<3;i++)
                r.at<float>(i)=(*this)[i];
            cv::Rodrigues(r,_G2F(cv::Range(0,3),cv::Range(0,3)));

            for(int i=0;i<3;i++)
                _G2F.at<float>(i,3)=(*this)[i+3];//
        }
    };
    
    //    std::map<unsigned int, Frame2Global> frame_transforms;
    std::map<uint32_t, Frame2Global> frame_transforms;

    class MappointError: public nanogo::Error<double>{
        vector<cv::Point3f> local_point_positions;

    public:
        void associateFrameTransform(Frame2Global* f2g, cv::Point3f p){
            push_back(f2g);
            local_point_positions.push_back(p);
        }
        void compute(Eigen::Matrix<double,Eigen::Dynamic,1> &e){
            e.resize(local_point_positions.size()*3);
            cv::Point3f mean(0,0,0);
            vector<cv::Point3f> pointsGlobal(size());
            for(size_t i=0;i<size();i++){
                pointsGlobal[i]=((Frame2Global*)at(i))->_G2F*local_point_positions[i];
                mean+= pointsGlobal[i];
            }
            mean/=float(size());
            for(size_t i=0;i<size();i++){
                e(i*3)= pointsGlobal[i].x-mean.x;
                e(i*3+1)= pointsGlobal[i].y-mean.y;
                e(i*3+2)= pointsGlobal[i].z-mean.z;
            }
        }
    };
    
    std::vector<MappointError> mappoint_errors;
    
    tslam::Map tslam_map;
    nanogo::Graph<double> op_graph;

public:
    SequenceOptimizer(tslam::Map &um);
    void optimize();
    void getTransforms(std::map<unsigned int,cv::Mat> &result);
};

#endif // SEQUENCEOPTIMIZER_H
