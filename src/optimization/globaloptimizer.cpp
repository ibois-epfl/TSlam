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
#include "globaloptimizer.h"
#include "globaloptimizer_g2o.h"
#include <basictypes/hash.h>
 //#ifdef USE_CVBA
//#include "globaloptimizer_cvba_pba.h"
//#include "globaloptimizer_cvba_ucosba.h"
//#endif
namespace tslam{
std::shared_ptr<GlobalOptimizer> GlobalOptimizer::create(string type){
    if(type.empty() || type=="g2o")
        return std::make_shared<GlobalOptimizerG2O>();


    else throw std::runtime_error("GlobalOptimizer::create could not load the required optimizer");
}


void GlobalOptimizer::saveToStream(std::ostream &str){

}

void GlobalOptimizer::readFromStream(std::istream &str){

}
uint64_t GlobalOptimizer::ParamSet::getSignature()const{
    Hash sig;
    sig+=fixFirstFrame;
    sig+=minStepErr;
    sig+=minErrPerPixel;//below this value, the optimization stops
    sig+=nIters;//number of iterations
    sig+=verbose;
    sig+=markersOptWeight;//importance of markers in the final error. Value in range [0,1]. The rest if assigned to points
    sig+=minMarkersForMaxWeight;
    sig+=InPlaneMarkers;
    sig+=markerSize ;
    sig.add(used_frames.begin(),used_frames.end());
    sig.add(fixed_frames.begin(),fixed_frames.end());
    return sig;
}

}
