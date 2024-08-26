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
    Frame &addKeyFrame(Frame *f, bool forceAddNewMarker = false);

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
    uint32_t numInitKFs=0;
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
};

}

#endif
