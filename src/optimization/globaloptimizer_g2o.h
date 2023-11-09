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
#ifndef _TSLAM_GLOBAL_OPTIMIZER_G2O_H_
#define _TSLAM_GLOBAL_OPTIMIZER_G2O_H_
#include "globaloptimizer.h"

namespace g2o{
class SparseOptimizer;
};

namespace tslam{

/**Performs a global optimization of points,markers and camera locations
 */
class   GlobalOptimizerG2O: public GlobalOptimizer{
public:
    void setParams(std::shared_ptr<Map> map, const ParamSet &p=ParamSet() );
    void optimize(bool *stopASAP=nullptr) ;
    void getResults(std::shared_ptr<Map> map);

    void optimize(std::shared_ptr<Map> map,const ParamSet &p=ParamSet() ) ;
    vector<std::pair<uint32_t,uint32_t>> getBadAssociations( ){return _badAssociations;}

    string getName()const{return "g2o";}


private:

    template<typename T>
    struct zeroinitvar{
        operator T& (){return val;}
        operator T ()const{return val;}
        void operator++(int){val++;}
        T val=0;
    };



    ParamSet _params;


    void saveToStream_impl(std::ostream &str){};
    void readFromStream_impl(std::istream &str){};
    std::shared_ptr<g2o::SparseOptimizer> Optimizer;


    uint64_t join(uint32_t a ,uint32_t b){
        uint64_t a_b;
        uint32_t *_a_b_16=(uint32_t*)&a_b;
        _a_b_16[0]=b;
        _a_b_16[1]=a;
        return a_b;
    }
    inline pair<uint32_t,uint32_t> separe(uint64_t a_b){         uint32_t *_a_b_16=(uint32_t*)&a_b;return make_pair(_a_b_16[1],_a_b_16[0]);}



    vector<std::pair<uint32_t,uint32_t> > _badAssociations;
    vector<float> _InvScaleFactors;
    struct edge_frameId_stereo{
        edge_frameId_stereo(void *f,uint32_t s,bool isSt, bool isMCam=false){
            first=f;
            second=s;
            isStereo=isSt;
            isMultiCam=isMCam;
        }
        void *first;
        uint32_t second;
        bool isStereo=false;
        bool isMultiCam=false;
    };

    struct markerInfo{
        void *vertex;
        int IdOptmz;
    };

    std::vector<std::vector< edge_frameId_stereo > > point_edges_frameId;
     std::vector<void* > marker_edges;
    std::map<uint32_t,markerInfo> marker_info;

    std::map<uint32_t, zeroinitvar<double> > frame_kpOptWeight;//for each frame, the total weight of the errors due to keypoints
    std::map<uint32_t, zeroinitvar<double> > frame_MarkerWeight;//for each frame, the total weight of the errors due to markers

    vector<uint32_t> usedFramesIdOpt,usedPointsIdOpt,usedMapPoints;
    vector<char> isFixedFrame;
    const uint32_t INVALID_IDX=std::numeric_limits<uint32_t>::max();
    const uint32_t INVALID_VISITED_IDX=std::numeric_limits<uint32_t>::max()-1;

    const char UNFIXED=0;
    const char FIXED_WITHOUTPOINTS=1;
    const char FIXED_WITHPOINTS=2;



    const float Chi2D = 5.99;
    const float Chi3D = 7.815;
    const float Chi4D = 9.49;
    const float Chi8D = 15.507;

    const float thHuber2D = sqrt(Chi2D);
    const float thHuber3D= sqrt(Chi3D);
    const float thHuber8D = sqrt(Chi8D);

};
}
#endif
