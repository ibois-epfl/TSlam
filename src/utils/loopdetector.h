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
#ifndef _TSLAM_LOOPDETECTOR_H
#define _TSLAM_LOOPDETECTOR_H
#include "map.h"
#include <thread>
namespace  tslam {
class BaseLoopDetector{
public:
    //Information about to the loop closure
    struct LoopClosureInfo{
        void clear(){
            optimPoses.clear();
         }

        bool foundLoop() const{ return optimPoses.size()!=0; }
        //reference frame where the loop is detected
        uint32_t curRefFrame=std::numeric_limits<uint32_t>::max();
        //the other side of the loop
        uint32_t matchingFrameIdx=std::numeric_limits<uint32_t>::max();
        //expected pose of current frame it is was estimated with the info from the other side of the loop
        cv::Mat expectedPos;
        //matches between the current frame an the map
        std::vector<cv::DMatch> map_matches;
        std::map<uint32_t, cv::Mat> optimPoses;//new optimized poses for the keyframes
        void toStream(ostream &rtr)const;
        void fromStream(istream &rtr);
        uint64_t getSignature();

    };



    virtual void setParams(std::shared_ptr<Map> map)=0;
    virtual LoopClosureInfo detectLoopFromMarkers(Frame &frame, int32_t curRefKf )=0;
    virtual LoopClosureInfo detectLoopFromKeyPoints(Frame &frame, int32_t curRefKf)=0;
    virtual void correctMap(const LoopClosureInfo &lcsol)=0;

 };

//A loop detector that does not do the job. Will never find a loop
class UselessLoopDetector:public BaseLoopDetector{

    void setParams(std::shared_ptr<Map> map) override{(void)(map);}
    LoopClosureInfo  detectLoopFromMarkers(Frame &frame, int32_t curRefKf )override{(void)(frame);(void)(curRefKf); return LoopClosureInfo();}
    LoopClosureInfo detectLoopFromKeyPoints(Frame &frame, int32_t curRefKf)override{(void)(frame);(void)(curRefKf);return LoopClosureInfo();}
    void correctMap(const LoopClosureInfo &lcsol)override{(void)(lcsol); }
};
//Real loop detector.
class LoopDetector:public BaseLoopDetector{
    std::shared_ptr<Map> TheMap;
public:

    void setParams(std::shared_ptr<Map> map) override;
    LoopClosureInfo  detectLoopFromMarkers(Frame &frame, int32_t curRefKf )override;
    LoopClosureInfo detectLoopFromKeyPoints(Frame &frame, int32_t curRefKf)override;
    void correctMap(const LoopClosureInfo &lcsol)override;

private:
    //obfuscate start
    vector<LoopClosureInfo>  detectLoopClosure_Markers(Frame & frame,int64_t curkeyframe);
    std::vector<LoopClosureInfo> detectLoopClosure_KeyPoints( Frame &frame, int32_t curRefKf);
    void solve(Frame &frame,   LoopClosureInfo &loopc_info);
    double testCorrection(const LoopClosureInfo &lcsol);
    void postLoopClosureCorrection(Frame &frame,const LoopClosureInfo &lcsol);
    //finds the best matches of the frame and the keyframe indicated assuming no initial knowledge of the pose
    vector<cv::DMatch> matchFrameToKeyFrame(std::shared_ptr<Map> map, const Frame&, uint32_t kf, float minDescDist,void*xfindex);
    //obfuscate end
};
}
#endif

