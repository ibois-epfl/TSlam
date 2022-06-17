/**
* This file is part of  UCOSLAM
*
* Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)
*
* UCOSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* UCOSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with UCOSLAM. If not, see <http://wwmap->gnu.org/licenses/>.
*/
#ifndef _UCOSLAM_LOOPDETECTOR_H
#define _UCOSLAM_LOOPDETECTOR_H
#include "map.h"
#include <thread>
namespace reslam {
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
    virtual LoopClosureInfo  detectLoopFromMarkers(Frame &frame, int32_t curRefKf )=0;
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
    vector<LoopClosureInfo>  detectLoopClosure_Markers(Frame & frame,int64_t curkeyframe);
    std::vector<LoopClosureInfo> detectLoopClosure_KeyPoints( Frame &frame, int32_t curRefKf);
    void solve(Frame &frame,   LoopClosureInfo &loopc_info);
    double testCorrection(const LoopClosureInfo &lcsol);
    void postLoopClosureCorrection(Frame &frame,const LoopClosureInfo &lcsol);
    //finds the best matches of the frame and the keyframe indicated assuming no initial knowledge of the pose
    vector<cv::DMatch> matchFrameToKeyFrame(std::shared_ptr<Map> map, const Frame&, uint32_t kf, float minDescDist,void*xfindex);
};
}
#endif

