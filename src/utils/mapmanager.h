/**
* This file is part of  TSLAM
*
* Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)
*
* TSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* TSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with TSLAM. If not, see <http://wwmap->gnu.org/licenses/>.
*/

#ifndef MapManager_H
#define MapManager_H

#include <memory>
#include <mutex>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include "basictypes/tsqueue.h"
#include "utils/loopdetector.h"
namespace  tslam {

//Class that runs in parallel taking care of the map
class Map;
class GlobalOptimizer;
class MapManager{

struct NewPointInfo{

    cv::Point3d pose;
    bool isStereo=false;
    vector<pair<uint32_t,uint32_t> > frame_kpt;
    float dist=std::numeric_limits<float>::max();
};

public:

    enum STATE { IDLE,WORKING,WAITINGFORUPDATE};

    MapManager();
    ~MapManager();
    void setParams(std::shared_ptr<Map> map,bool EnableLoopClosure);
    bool hasMap()const;
     void start();
    void stop();
    void reset();
    //call whenever a new frame is avaiable.
    //return 0 if nothing is done
    int newFrame(Frame &kf,  int32_t curkeyFrame);
    Frame &addKeyFrame(Frame *f);

    //applies the changes to map require tracking to be stoped, such as removing frames  or points
    bool mapUpdate(void);
    //returns true if a big change in the map has happened. Call after mapUpdate only and before next call to newFrame
    bool bigChange()const;
    //get the pose of the last frame added
    Se3Transform getLastAddedKFPose();
    uint32_t getLastAddedKeyFrameIdx()const;
    void toStream(std::ostream &str) ;
    void fromStream(istream &str);

    uint64_t getSignature();

private:
    //obfuscate start
    void runThread();
    void mainFunction();
    bool mustAddKeyFrame(const Frame &frame_in , uint32_t curKFRef);
    bool mustAddKeyFrame_Markers(const Frame & frame_in , uint32_t curKFRef);
    bool mustAddKeyFrame_KeyPoints(const Frame & frame_in, uint32_t curKFRef );
    bool mustAddKeyFrame_stereo(const Frame &frame_in,uint32_t curKFRef);

    std::vector<NewPointInfo> createNewPoints(Frame &NewFrame , uint32_t nn=20,uint32_t maxPoints=std::numeric_limits<uint32_t>::max());
    std::list<NewPointInfo> createCloseStereoPoints(Frame & newFrame);
    vector<uint32_t>  mapPointsCulling();
    void  localOptimization(uint32_t _newKFId,int nIters=5);
    void  globalOptimization(int niters=10 );

    set<uint32_t> keyFrameCulling(uint32_t keyframe_idx, bool checkRedundancy = true);
    set<uint32_t> keyFrameCulling_Markers(uint32_t keyframe_idx);
    set<uint32_t> keyFrameCulling_KeyPoints(uint32_t keyframe_idx, int max=1);


    vector<uint32_t> getMatchingFrames(Frame &NewFrame, size_t n)    ;
    //  void correctLoop(Frame &f, int32_t curRefKf, uint32_t matchingFrameIdx, cv::Mat estimatedPose);
    //adds the keyframe, remove useless points, and do global optimization
    void loopClosurePostProcessing(Frame &frame, const LoopDetector::LoopClosureInfo &lci);
    vector<uint32_t> searchInNeighbors(Frame &mpCurrentKeyFrame);


    std::shared_ptr<BaseLoopDetector> createLoopDetector(bool loopClosureEnabled);
    //////////////////////////////////////////////////
    /// variables
    //////////////////////////////////////////////////

    std::thread _TThread;
    std::mutex mutex_addKf;
    TSQueue<Frame*>  keyframesToAdd;

    std::shared_ptr<Map> TheMap;
    bool _mustAddKeyFrame=false;
    int nFramesAnalyzedWithoutAddingKF=0;
    std::atomic<STATE> _curState;
    bool mustExit=false;
     uint32_t _CurkeyFrame=std::numeric_limits<uint32_t>::max();//current keyframe of the tracker
    vector<uint32_t> PointsToRemove;
    set<uint32_t> KeyFramesToRemove;
    queue<uint32_t> newInsertedKeyFrames;
    //we need to save now
    std::shared_ptr<GlobalOptimizer> Gopt;
    std::map<uint32_t,uint32_t > youngKeyFrames;
    Se3Transform _lastAddedKFPose;
    bool bigChangeHasHappen=false;
    std::shared_ptr<BaseLoopDetector> _TheLoopDetector;
    LoopDetector::LoopClosureInfo _LoopClosureInfo;
    uint32_t _lastAddedKeyFrame=std::numeric_limits<uint32_t>::max();
    bool _hasMapBeenScaled=false;
    bool _hurryUp=false;//activated when a new keyframe should be added
    bool _loopClosureEnabled=true;
    //obfuscate end
};

}

#endif
