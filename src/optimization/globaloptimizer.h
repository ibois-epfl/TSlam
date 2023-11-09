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
#ifndef _TSLAM_GlobalOptimizerH_
#define _TSLAM_GlobalOptimizerH_
#include "map.h"
#include <unordered_set>
namespace tslam{
class System;

/**Base class for global optimization of points,markers and camera locations
 */
class GlobalOptimizer{
 public:

    struct ParamSet {
        ParamSet(bool Verbose=false):verbose(Verbose){}
        std::unordered_set<uint32_t> used_frames;//which are used. If empty, all
        std::set<uint32_t> fixed_frames;//which are set as not movable
        bool fixFirstFrame=true;
        float minStepErr=1e-2;
        float minErrPerPixel=0.25;//below this value, the optimization stops
        int nIters=100;//number of iterations
        bool verbose=false;
        float markersOptWeight=1.0;//importance of markers in the final error. Value in range [0,1]. The rest if assigned to points
        int minMarkersForMaxWeight=5;
        bool InPlaneMarkers=false;
          //---do not use from here
        float markerSize=-1;
        uint64_t getSignature()const;
    };

    //set the required params
    virtual void setParams(std::shared_ptr<Map>   map, const ParamSet &ps )=0;
    virtual void optimize(bool *stopASAP=nullptr) =0;
    virtual void getResults(std::shared_ptr<Map> map)=0;

    //one funtion to do everything
    virtual void optimize(std::shared_ptr<Map> map,const ParamSet &p=ParamSet() )=0;
    virtual string getName()const=0;

    //returns a vector of mapPointId,FrameId indicating the bad associations that should be removed
    virtual vector<std::pair<uint32_t,uint32_t>> getBadAssociations( ){return {};}


    static std::shared_ptr<GlobalOptimizer> create(string type="");

    void saveToStream(std::ostream &str);
    void readFromStream(std::istream &str);
protected:
    virtual void saveToStream_impl(std::ostream &str)=0;
    virtual void readFromStream_impl(std::istream &str)=0;
};
}
#endif
