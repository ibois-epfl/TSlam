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
#include "sequenceoptimizer.h"
#include <iostream>

using namespace std;

SequenceOptimizer::SequenceOptimizer(reslam::Map &um/*,std::map<unsigned int,std::map<unsigned int,cv::Point3f>> &local_keypoint_positions*/){
    reslam::Map& ucoslam_map=um;

    for(reslam::Frame &f: ucoslam_map.keyframes){
        frame_transforms[f.idx].setParams(f.pose_f2g.inv(), f.idx);
    }
    for(auto &f:frame_transforms){
        op_graph.add(&frame_transforms[f.first]);
    }

    mappoint_errors.reserve(ucoslam_map.map_points.size());
    for(reslam::MapPoint mp:ucoslam_map.map_points){
        if(mp.isBad())
            continue;
        std::vector<std::pair<uint32_t,uint32_t>> assosiated_frames=mp.getObservingFrames();

        MappointError me;
        bool has_valid_keypoints=false;
        for( std::pair<uint32_t,uint32_t> &p:assosiated_frames){
            unsigned int frame_index=p.first;
//            cout<<frame_index<<endl;
            unsigned int frame_keypoint_index=p.second;
            if(ucoslam_map.keyframes[frame_index].getDepth(frame_keypoint_index)==0)
                continue;

            has_valid_keypoints=true;
//            auto keypoint=ucoslam_map.keyframes[frame_index].und_kpts[frame_keypoint_index];
            me.associateFrameTransform(&frame_transforms[frame_index],ucoslam_map.keyframes[frame_index].get3dStereoPoint(frame_keypoint_index)/*local_keypoint_positions[frame_index][frame_keypoint_index]*/);
        }
        if(has_valid_keypoints){
            mappoint_errors.push_back(me);
            op_graph.add(&(mappoint_errors.back()));
        }
    }
}

void SequenceOptimizer::optimize(){
    nanogo::SparseGraphSolver<double> sgs;
    nanogo::SparseGraphSolver<double>::Params sgsps;
    sgsps.verbose=true;
    sgs.solve(this->op_graph,sgsps);
}

void SequenceOptimizer::getTransforms(std::map<unsigned int,cv::Mat> &result){
    result.clear();
    for(auto frame_transform:frame_transforms){
        frame_transform.second.prepare();
        frame_transform.second._G2F.copyTo(result[frame_transform.first]);
    }
}
