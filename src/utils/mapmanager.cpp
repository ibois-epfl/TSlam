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
#include "mapmanager.h"
#include "basictypes/osadapter.h"
#include "map.h"
#include "system.h"
#include "optimization/ippe.h"
#include "basictypes/misc.h"
#include "basictypes/io_utils.h"
#include "basictypes/timers.h"
#include "optimization/globaloptimizer.h"
#include "optimization/pnpsolver.h"
#include "basictypes/minmaxbags.h"
#include <xflann/xflann.h>
#include "basictypes/hash.h"
#include "framematcher.h"
#ifdef USE_OMP
#include <omp.h>
#endif
namespace tslam{

MapManager::MapManager(){
    _curState=IDLE;
    _TheLoopDetector=createLoopDetector(_loopClosureEnabled);
 }
MapManager::~MapManager(){
    stop();
}
void MapManager::setParams(std::shared_ptr<Map> map, bool EnableLoopClosure){
    TheMap=map;
    numInitKFs=TheMap->keyframes.size();
    _loopClosureEnabled=EnableLoopClosure;
    _TheLoopDetector=createLoopDetector(_loopClosureEnabled);
    _TheLoopDetector->setParams(map);
}

uint32_t MapManager::getLastAddedKeyFrameIdx()const{return _lastAddedKeyFrame;}
bool MapManager::hasMap()const{ return !(!TheMap);}

std::shared_ptr<BaseLoopDetector> MapManager::createLoopDetector(bool loopClosureEnabled){
     if(loopClosureEnabled) return std::make_shared<LoopDetector>();
    else return std::make_shared<UselessLoopDetector>();
}



int MapManager::newFrame(Frame &kf, int32_t curkeyFrame  ){
    __TSLAM_ADDTIMER__
            _CurkeyFrame=curkeyFrame;
    nFramesAnalyzedWithoutAddingKF++;
    bigChangeHasHappen=false;
    int returnValue=0;

    if (  _curState.load()==IDLE ){
#pragma message "warning : in non-sequential mode detected markers in loop closure are not proceesed properly?"
        if(System::getParams().reLocalizationWithMarkers && System::getParams().enableLoopClosure ){
            cout << "Detecting loop closure from markers..." << endl;
            _LoopClosureInfo=_TheLoopDetector->detectLoopFromMarkers(kf,curkeyFrame);
            if (_LoopClosureInfo.foundLoop()){
                _debug_msg_("Loop closure from markers");
                _TheLoopDetector->correctMap(_LoopClosureInfo);//
                loopClosurePostProcessing(kf,_LoopClosureInfo);//the keyframe is added here
                bigChangeHasHappen=true;
                __TSLAM_TIMER_EVENT__("detectLoop");
                returnValue=2;
            }
        }
        else{
            __TSLAM_TIMER_EVENT__("detectLoop");
            if ( mustAddKeyFrame(kf,curkeyFrame)   ){
                nFramesAnalyzedWithoutAddingKF=0;
                __TSLAM_TIMER_EVENT__("mustAddKeyFrame");
                Frame *newKF =new Frame(kf);
                keyframesToAdd.push(   newKF );
                //if the thread is not running, we work in synchronous mode
                if (!_TThread.joinable())
                    mainFunction();
                __TSLAM_TIMER_EVENT__("mainFunction");
                returnValue=1;
            }
            else{
                __TSLAM_TIMER_EVENT__("mainFunction");
            }
        }
    }
    else{//in the middle of something
        if ( mustAddKeyFrame(kf,curkeyFrame)   ){
         //   _hurryUp=true;
        }
        if(System::getParams().reLocalizationWithMarkers && System::getParams().enableLoopClosure) {
            _LoopClosureInfo = _TheLoopDetector->detectLoopFromMarkers(kf, curkeyFrame);
            if (_LoopClosureInfo.foundLoop()) {
                //   _hurryUp=true;
            }
        }
    }
    return returnValue;
}



bool MapManager::mapUpdate(){

    if (_curState!=WAITINGFORUPDATE)
        return  false  ;

    _curState=WORKING;
    __TSLAM_ADDTIMER__

    TheMap->lock(__FUNCTION__,__FILE__,__LINE__);

    if (System::getParams().enableLoopClosure && _LoopClosureInfo.foundLoop()){
        //get the detection and correct the map
        _TheLoopDetector->correctMap(_LoopClosureInfo);
        loopClosurePostProcessing(TheMap->keyframes[_lastAddedKeyFrame],_LoopClosureInfo  );
        bigChangeHasHappen=true;
        __TSLAM_TIMER_EVENT__  ("Loop closure correction");

    }
    else{
        vector<std::pair<uint32_t,uint32_t>> BadAssociations;
        if (Gopt){//if we jsut wake up, from a loadFromStream, this object is not created
             Gopt->getResults(TheMap);
             BadAssociations=Gopt->getBadAssociations();
            Gopt=nullptr;
        }
        TheMap->removeBadAssociations(BadAssociations,System::getParams().minNumProjPoints);
        __TSLAM_TIMER_EVENT__  ("Removing bad associations");
    }
    //complete point removal (if they remain after previous operations)
    for(auto p:PointsToRemove)
        if (TheMap->map_points.is(p))
            TheMap->removePoint(p);
    PointsToRemove.clear();
    __TSLAM_TIMER_EVENT__  ("Removing bad points");

    TheMap->removeKeyFrames(KeyFramesToRemove,System::getParams().minNumProjPoints);
    __TSLAM_TIMER_EVENT__  ("Removing keyframes");

    for(auto kf:KeyFramesToRemove) youngKeyFrames.erase(kf);

    if(_hasMapBeenScaled){
        bigChangeHasHappen=true;
    }

    _lastAddedKFPose=TheMap->keyframes[_lastAddedKeyFrame].pose_f2g;



     TheMap->removeWeakConnections(_CurkeyFrame,8);


    TheMap->unlock(__FUNCTION__,__FILE__,__LINE__);
    //---------------------------------------------------------
    // UNLOCK
    //---------------------------------------------------------
    //  assert(TheMap->checkConsistency());
    PointsToRemove.clear();
    KeyFramesToRemove.clear();
    _curState=IDLE;
    return true;

}


void MapManager::start(){
    if (_TThread.joinable()) return;//is running
    mustExit=false;
    _TThread= std::thread([this]{ this->runThread();});

}
void MapManager::stop(){
    if (_TThread.joinable()) { //is running
        mustExit=true;
        _hurryUp=false;
        keyframesToAdd.push(NULL);//this will wakeup in mainFuntion if sleeping
        _TThread.join();
    }

}
void MapManager::reset(){
    if (_TThread.joinable()) { //is running
        mustExit=true;
        keyframesToAdd.push(NULL);//this will wakeup in mainFuntion if sleeping
        _TThread.join();
    }
    mustExit=false;
    keyframesToAdd.clear();
    _curState=IDLE;
    TheMap.reset();
    _CurkeyFrame=std::numeric_limits<uint32_t>::max();//current keyframe of the tracker


    //we need to save now
    Gopt.reset();
    PointsToRemove.clear();
    KeyFramesToRemove.clear();
    std::map<uint32_t,uint32_t > youngKeyFrames;


    _TheLoopDetector=createLoopDetector(_loopClosureEnabled);
     _LoopClosureInfo=LoopDetector::LoopClosureInfo();
     _lastAddedKeyFrame=std::numeric_limits<uint32_t>::max();
     _hasMapBeenScaled=false;
     _hurryUp=false;


}



Frame& MapManager::addKeyFrame(Frame *newPtrFrame){
    auto getNOFValidMarkers=[this](){
        int nValidMarkers=0;
        for(auto &m:TheMap->map_markers)
            if (m.second.pose_g2m.isValid()) nValidMarkers++;
        return nValidMarkers;
    };

    // in localizeOnly mode, filter out the marker that is not in the map
    if(System::getParams().localizeOnly){
        std::vector<tslam::MarkerObservation> filteredMarkers;
        for(auto m : newPtrFrame->markers) {
            if (TheMap->map_markers.count(m.id)!=0){
                filteredMarkers.push_back(m);
            }
        }
        newPtrFrame->markers = filteredMarkers;
    }


    ///ADD KEYFRAME
    __TSLAM_ADDTIMER__
    Frame &newFrame=TheMap->addKeyFrame(*newPtrFrame);
    newInsertedKeyFrames.push(newFrame.idx);
    _lastAddedKeyFrame=newFrame.idx;

    __TSLAM_TIMER_EVENT__("Add frame  ");
    youngKeyFrames.insert({newFrame.idx,0});
    //remove old frames from here
    vector<uint32_t> toRemove;
    for(auto &kf:youngKeyFrames){
        kf.second++;
        if (kf.second>3)  toRemove.push_back(kf.first);
    }
    for(auto r:toRemove) youngKeyFrames.erase(r);

    if(System::getParams().KPNonMaximaSuppresion)
        newFrame.nonMaximaSuppresion();
    __TSLAM_TIMER_EVENT__("NonMaximaSuppresion");
    //add to the set of unstable frames
    //UPDATE EXISTING MAP POINTS WITH OBSERVATIONS IN THIS FRAME
    int nObsPoins=0;
    for(size_t i=0;i<newFrame.ids.size();i++){
        if (newFrame.ids[i]!=std::numeric_limits<uint32_t>::max()){
              TheMap-> addMapPointObservation(newFrame.ids[i],newFrame.idx,i);
              nObsPoins++;
        }
    }

    __TSLAM_TIMER_EVENT__(" Map observations:" );


    //Analyze if the case in which the first marker of the scene is spotted. Then, the map would need to change scale
    _hasMapBeenScaled=false;
    bool mayNeedChangeMapScale=false;
    if( getNOFValidMarkers()==0 && TheMap->map_points.size()!=0) mayNeedChangeMapScale=true;

    //now, go with the markers
    for(size_t m=0;m< newFrame.markers.size();m++){
        //add marker if not yet
        auto &map_marker= TheMap->addMarker(newFrame.markers[m]);
        //add observation
        TheMap->addMarkerObservation(map_marker.id,newFrame.idx);
        //has the marker valid info already?

        if (!map_marker.pose_g2m.isValid()){//no, see if it is possible to add the pose now with current data

             //if is not the first marker and is allowed oneframe initialization
            if (!mayNeedChangeMapScale  && System::getParams().aruco_allowOneFrameInitialization )  {
                //Is the detection reliable (unambiguos)? If so, assign pose
                if( newFrame.markers[m].poses.err_ratio> System::getParams().aruco_minerrratio_valid)
                    map_marker.pose_g2m=  newFrame.pose_f2g.inv()*newFrame.markers[m].poses.sols[0];
            }
        }

        auto FramesDist=[](Se3Transform &a,Se3Transform &b){
            auto ta=a(cv::Range(0,3),cv::Range(3,4));
            auto tb=b(cv::Range(0,3),cv::Range(3,4));
            return cv::norm(ta-tb);
        };
        //if the above method did not succed, can we do the same using multiple views????
        if (!map_marker.pose_g2m.isValid() && map_marker.frames.size()>=size_t(System::getParams().aruco_minNumFramesRequired) ){
            //check how many views with enough distance are
            vector<uint32_t> vframes( map_marker.frames.begin(),map_marker.frames.end());
            std::vector<bool> usedFrames(vframes.size(),false);
            std::vector<uint32_t> farEnoughViews;farEnoughViews.reserve(map_marker.frames.size());
            for(size_t i=0;i<vframes.size() ;i++){
                //find farthest
                if (!usedFrames[i]){
                    pair<int,float> best(-1,std::numeric_limits<float>::lowest());
                    for(size_t j=i+1;j<vframes.size();j++){
                        if( !usedFrames[j]){
                            float d=FramesDist( TheMap->keyframes[vframes[i]].pose_f2g,TheMap->keyframes[vframes[j]].pose_f2g);
                            if (d>System::getParams().minBaseLine && d> best.first)
                                best={j,d};
                        }
                    }
                    if (best.first!=-1){
                        assert(std::find(farEnoughViews.begin(),farEnoughViews.end(),vframes[i])==farEnoughViews.end());
                        assert(std::find(farEnoughViews.begin(),farEnoughViews.end(),vframes[best.first])==farEnoughViews.end());
                        farEnoughViews.push_back(vframes[i]);
                        farEnoughViews.push_back(vframes[best.first]);
                        usedFrames[i]=true;
                        usedFrames[best.first]=true;
                    }
                }
            }

            if (farEnoughViews.size()>=size_t(System::getParams().aruco_minNumFramesRequired)){//at least x views to estimate the pose using multiple views

                vector<tslam::MarkerObservation> marker_views;
                vector<se3> frame_poses;
                for(auto f:farEnoughViews){
                    marker_views.push_back(TheMap->keyframes[f].getMarker(map_marker.id));
                    frame_poses.push_back(TheMap->keyframes[f].pose_f2g);
                }
                auto pose=ARUCO_bestMarkerPose(marker_views,frame_poses,newFrame.imageParams.undistorted());
                if (!pose.empty()){
                    _debug_msg_("added marker "<<map_marker.id<<" using multiple views");
                    map_marker.pose_g2m=pose;
                }
            }
        }
    }
    /////////////////////////////////////////////////////////////////////////////
    ///     MAP RESCALING BECAUSE OF NEW MARKER FOUND
    //if there were no valid markers before, but points, and a new marker is found,
    //then, there is the need to establish
    if (mayNeedChangeMapScale&& getNOFValidMarkers()>0){
            //the pose of a marker has been established. It is required to scale the map by
            //findding correspondences between the marker system and the points
            //to do so, analyze the points into the marker. They are employed to scale
        //find the 3d points into the marker in this image
        pair<double,double> avrg_scale(0,0);
        for(auto &m:newFrame.markers){
            auto &mapMarker=TheMap->map_markers.at(m.id);
            if (!mapMarker.pose_g2m.isValid()) continue;
            //determine center and maximum distance to it
            cv::Point2f center(0,0);
            for(auto p:m.und_corners)center+p;
            center*=1./4.;
            //now max dist
            double maxDist=std::numeric_limits<double>::min();
            for(auto p:m.und_corners)   maxDist=std::max( cv::norm(center-p),maxDist);
            vector<uint32_t> p3dis=newFrame.getIdOfPointsInRegion(center,maxDist);
            if ( p3dis.size()<5)continue;//too few points
            //get the scale
            //get the average distance to the camera
            double distSum=0;
            for(auto pid:p3dis){
                //move the point to the frame and compute distance to camera
                distSum+=   cv::norm( newFrame.pose_f2g*TheMap->map_points[pid].getCoordinates());
            }
            double avrgPointDist=distSum/double(p3dis.size());
            //compute the distance of the marker to the frame
            cv::Mat f2m=newFrame.pose_f2g*mapMarker.pose_g2m;
            double frameDist=cv::norm(  f2m.rowRange(0,3).colRange(3,4));
            avrg_scale.first+=frameDist/avrgPointDist;
            avrg_scale.second++;
        }

        if ( avrg_scale.second==0){//cant scale because no evidences found. Remove pose of the markers
            for(auto &m:TheMap->map_markers)
                m.second.pose_g2m=se3();
        }
        else{//reescale the whole map
            // Scale points
            double scaleFactor=avrg_scale.first/avrg_scale.second;
            TheMap->scale(scaleFactor);
            globalOptimization(10);
            _hasMapBeenScaled=true;

        }
    }

    return newFrame;
}

void MapManager::mainFunction(){
    _hurryUp=false;

    //first check if any new frame to be inserted
    Frame    *newPtrFrame;
    keyframesToAdd.pop(newPtrFrame);


    if(newPtrFrame==NULL) return;//a NULL frame mean, leave
    _curState=WORKING;

    __TSLAM_ADDTIMER__

    ///ADD KEYFRAME

    TheMap->lock(__FUNCTION__,__FILE__,__LINE__);
    Frame &newFrame=addKeyFrame(newPtrFrame);
    _debug_msg_("");
    delete newPtrFrame;
    __TSLAM_TIMER_EVENT__("add keyframe");

    //start the thread to search for loop closures
    if(System::getParams().enableLoopClosure && System::getParams().reLocalizationWithKeyPoints)
        _LoopClosureInfo=_TheLoopDetector->detectLoopFromKeyPoints(newFrame,_CurkeyFrame);


    __TSLAM_TIMER_EVENT__("loop detection ");

    _debug_msg_("");
    __TSLAM_TIMER_EVENT__("update existing points ");
    TheMap->unlock(__FUNCTION__,__FILE__,__LINE__);



//     assert(TheMap->checkConsistency(true ));
    _debug_msg_("");


    //FIND Points to remove
    PointsToRemove=mapPointsCulling();
    TheMap->removePoints(PointsToRemove.begin(),PointsToRemove.end(),false);//preremoval
     __TSLAM_TIMER_EVENT__("map point culling and removal");

    TheMap->lock(__FUNCTION__,__FILE__,__LINE__);
    /// ADD NEW MAPPOINTS

    int nn=20;//how many neigbors are search in monocular mode
    //STEREO POINTS
    if ( newFrame.imageParams.isStereoCamera()){
        nn=5;
        for(const auto &nmp:createCloseStereoPoints(newFrame)){
            auto &mPoint=TheMap->addNewPoint(newFrame.fseq_idx);
            mPoint.setStereo(true);
            mPoint.setCoordinates(nmp.pose);
            for(auto obs:nmp.frame_kpt)
                TheMap->addMapPointObservation(mPoint.id,obs.first,obs.second);
        }
    }

    //NONSTEREO POINTS
        auto newPoints=createNewPoints(newFrame,nn,System::getParams().maxNewPoints);
        _debug_msg_("Added "<<newPoints.size()<<" new points");
        for(const auto &nmp:newPoints){
            auto &mPoint=TheMap->addNewPoint(newFrame.fseq_idx);
            mPoint.setCoordinates(nmp.pose);
            for(auto obs:nmp.frame_kpt)
                TheMap->addMapPointObservation(mPoint.id,obs.first,obs.second);
        }

    TheMap->unlock(__FUNCTION__,__FILE__,__LINE__);


    __TSLAM_TIMER_EVENT__("Add   points " );

    //if have time, refine look for more matches in the neigbors
     if (keyframesToAdd.empty() && System::getParams().detectKeyPoints ){
        TheMap->lock(__FUNCTION__,__FILE__,__LINE__);
        auto ptremove=searchInNeighbors(newFrame);
        PointsToRemove.insert(PointsToRemove.end(),ptremove.begin(),ptremove.end());
        __TSLAM_TIMER_EVENT__("Search in neighbors ");
        TheMap->unlock(__FUNCTION__,__FILE__,__LINE__);
      //  assert(TheMap->checkConsistency( ));
    }

    //---------------------------------------------------------
    // UNLOCK
    //---------------------------------------------------------


    __TSLAM_TIMER_EVENT__("map point culling  analysis");
    //how many bad points and to remove
    {
    int nBad=0;
    for(const auto &mp:TheMap->map_points)
        if (mp.isBad())nBad++;
     }
    //Local optimization
    if (!System::getParams().localizeOnly && !_hurryUp && TheMap->keyframes.size()>1 ){
        localOptimization(newFrame.idx);
        __TSLAM_TIMER_EVENT__("Local optimization");
    }
    //Keyframe culling
    if(!_hurryUp){
        KeyFramesToRemove=keyFrameCulling(newFrame.idx);
        for(auto kf:KeyFramesToRemove)
            TheMap->keyframes[kf].setBad(true);
    }

    // if there is > the specified number of new added frame, remove them to maintain the fps
    if (System::getParams().localizeOnly) {
        const int maxNewKF = 20;
        while (TheMap->keyframes.size() - numInitKFs > maxNewKF) {
            auto numKFtoRemove = TheMap->keyframes.size() - numInitKFs - maxNewKF;
            auto kfIDtoRemove = newInsertedKeyFrames.front();
            int counter = 0;
            while(!newInsertedKeyFrames.empty() && counter < numKFtoRemove) {
                if (TheMap->keyframes.is(kfIDtoRemove) && kfIDtoRemove != _lastAddedKeyFrame) {
                    TheMap->keyframes[kfIDtoRemove].setBad(true);
                    KeyFramesToRemove.insert(kfIDtoRemove);
                    counter++;
                }
                newInsertedKeyFrames.pop();
            }
        }
    }


    _curState=WAITINGFORUPDATE;
  //  assert(TheMap->checkConsistency());

}

//get the pose of the last frame added
Se3Transform MapManager::getLastAddedKFPose(){
return _lastAddedKFPose;
}

bool MapManager::bigChange() const{
    return bigChangeHasHappen;
}

void MapManager::runThread(){


    while(!mustExit){       
        mainFunction();
    }
}



set<uint32_t> MapManager::keyFrameCulling(uint32_t keyframe_idx, bool checkRedundancy){

    __TSLAM_ADDTIMER__

    set<uint32_t> KFtoRemove;

    //both markers and key points
    if( System::getParams().detectMarkers && TheMap->map_markers.size() != 0 ){ //must be in both
        vector<uint32_t> NotRedundant;

        KFtoRemove=keyFrameCulling_KeyPoints(keyframe_idx);
         //check the redundant keyframe in the marker sense: the same markers are already observed in another keyframe
        for(auto kf:KFtoRemove){
            const auto &ThisKFrame=TheMap->keyframes[kf] ;

            //get all keyframes in which the markers of this are observed
            std::set<uint32_t> allFrames;
            for(const auto &m:ThisKFrame.markers)
                for(auto f:TheMap->map_markers[m.id].frames)
                    allFrames.insert(f);

            allFrames.erase(kf);//remove this
            //now, see if this is redundant
            bool isRedundant=false;
            for(auto fidx:allFrames){
                int nMarkersCommon=0;
                for(const auto &m:TheMap->keyframes[fidx].markers)
                    if(ThisKFrame.getMarkerIndex(m.id)!=-1) nMarkersCommon++;
                if (nMarkersCommon==ThisKFrame.markers.size()) {
                    isRedundant=true;
                    break;
                }
            }
            if (!isRedundant)
                NotRedundant.push_back(kf);
        }
        //now, exlude the non redundant ones
        for(auto f:NotRedundant)
            KFtoRemove.erase(f);

    }
    //only keypoints
    else if (System::getParams().detectKeyPoints) KFtoRemove=keyFrameCulling_KeyPoints(keyframe_idx);
    //only markers
    else if (System::getParams().detectMarkers) KFtoRemove= keyFrameCulling_Markers(keyframe_idx);

    return KFtoRemove;

}

set<uint32_t> MapManager::keyFrameCulling_Markers(uint32_t keyframe_idx){

     auto join=[](uint32_t a ,uint32_t b){
        if( a>b)swap(a,b);
        uint64_t a_b;
        uint32_t *_a_b_16=(uint32_t*)&a_b;
        _a_b_16[0]=b;
        _a_b_16[1]=a;
        return a_b;
    };

     //auto separe=[](uint64_t a_b){  uint32_t *_a_b_16=(uint32_t*)&a_b;return  make_pair(_a_b_16[1],_a_b_16[0]);};


    //find the neightbors
    auto neigh=TheMap->TheKpGraph.getNeighbors(keyframe_idx);
    //remove first frame if it is in
    neigh.erase(TheMap->keyframes.front().idx);




     std::map<uint64_t,float> frame_distances;
    //compute distances between frames
    vector<uint32_t> vneigh(neigh.begin(),neigh.end());
    for(size_t i=0;i<vneigh.size();i++){
        const auto&fi=TheMap->keyframes[vneigh[i]];
        for(size_t j=i+1;j<vneigh.size();j++){
            const auto&fj=TheMap->keyframes[vneigh[j]];
          //  frame_distances[join(vneigh[i],vneigh[j])]= fi.pose_f2g.t_dist(fj.pose_f2g);
            frame_distances[join(vneigh[i],vneigh[j])]= cv::norm( fi.pose_f2g.getTvec() -fj.pose_f2g.getTvec());
            }
    }


    //first, remove frames dominated by others


    //for each marker, select a equdistant set of views


    ///determine the set of markers visible in the neighbors
    std::map<uint32_t,set<uint32_t> > marker_frames;//for each visible marker, the views it is seen it
    for(auto fidx:neigh){
        for(auto m:TheMap->keyframes[fidx].markers)
            marker_frames[m.id].insert(fidx);
    }


    auto distanceToFrames=[&](uint32_t fidx,const set<uint32_t> &frames){
            float dist=0;
            for(auto f2idx: frames){
                if (f2idx!=fidx )
                    dist+=frame_distances[join(fidx,f2idx)];
            }
            return dist;
    };

      ///for each marker, select a subset of frames (the most equdistant ones)
    std::map<uint32_t,set<uint32_t> > marker_selected_frames;//for each visible marker, the views it is seen it
     for(auto mf:marker_frames){
        if (  mf.second.size()<=size_t(System::getParams().maxVisibleFramesPerMarker)){
            marker_selected_frames[mf.first].insert( mf.second.begin(),mf.second.end());
        }
        else{
            //how many will be selected? (70%)


            //find the first two frames which are most equidistant
            vector<uint32_t> vframes(mf.second.begin(),mf.second.end());
            pair<size_t,size_t> bestIdx;float maxD=std::numeric_limits<float>::lowest();
            for(size_t i=0;i<vframes.size();i++){
                for(size_t j=i+1;j<vframes.size();j++){
                    auto dist= frame_distances[ join(vframes[i],vframes[j]) ];
                    if ( dist>maxD){
                        bestIdx={vframes[i],vframes[j]};
                        maxD=dist;
                    }
                }
            }
            //the best ones are in bestIdx
            marker_selected_frames[mf.first]. insert( bestIdx.first);
            marker_selected_frames[mf.first]. insert( bestIdx.second);
            //now, keep adding. The next is always the farthest from the current elements in the set
            while(marker_selected_frames[mf.first].size()<size_t(System::getParams().maxVisibleFramesPerMarker)){
                std::pair<uint32_t,float> best(0, std::numeric_limits<float>::lowest());
                for(size_t i=0;i<vframes.size();i++){
                    if ( marker_selected_frames[mf.first].count(vframes[i])==0){
                        auto d=distanceToFrames(vframes[i],marker_selected_frames[mf.first]);
                        if (d>best.second) best={vframes[i],d};
                    }
                }
                //add it
                assert( std::find(marker_selected_frames[mf.first].begin(),marker_selected_frames[mf.first].end(),best.first)==marker_selected_frames[mf.first].end());
                marker_selected_frames[mf.first]. insert(best.first);
            }
        }

    }

    //now, let us remove the elements not in the set of selected
    std::set<uint32_t> selected;
    for(auto ms:marker_selected_frames)
        selected.insert(ms.second.begin(),ms.second.end());
    //remove the rest
    std::set<uint32_t> toremove;
    for(auto fidx:neigh)    //remove neighbors not in the set
        if( selected.count(fidx)==0) toremove.insert(fidx);

    return toremove;

}




set<uint32_t> MapManager::keyFrameCulling_KeyPoints(uint32_t keyframe_idx,int max){
    __TSLAM_ADDTIMER__
     set<uint32_t> framesToRemove;

     if (TheMap->keyframes.size()<size_t(System::getParams().minNumProjPoints)) return {};
     //find the neightbors
    auto neigh=TheMap->TheKpGraph.getNeighbors(keyframe_idx);

    neigh.erase(0);///exclude first frame //and the second?
    neigh.erase(1);///exclude first frame //and the second?

    //exlcude too young frames
    for(auto ykf:youngKeyFrames)
        neigh.erase(ykf.first);

    __TSLAM_TIMER_EVENT__("getN");

    vector<pair<float,uint32_t> > KeyFramesThatCanBeRemoved;//candidates for removal and its percentage of redundancy

    int thresObs=System::getParams().minNumProjPoints;
    for(auto fidx:neigh){
        int nRedundant=0,nPoints=0;
         auto &frame=TheMap->keyframes[fidx];
        if (frame.isBad() )continue;

        for(size_t i=0;i<frame.ids.size();i++){
            if (frame.ids[i]!=std::numeric_limits<uint32_t>::max()){
                auto &mp=TheMap->map_points[frame.ids[i]];
                if (mp.isBad())continue;//to be removed
                nPoints++;
                //check point projections in the other frames and see if this is redundant
                int nObs=0;//number of times point observed in a finer scale in another frame
                if( mp.getNumOfObservingFrames()>size_t(thresObs)){
                    for(const auto &f_i:mp.getObservingFrames()){
                        if (f_i.first!=fidx && !TheMap->keyframes[f_i.first].isBad() )//not in a frame to be removed
                            if  ( TheMap->keyframes[f_i.first].und_kpts[f_i.second].octave<=frame.und_kpts[i].octave){
                                nObs++;
                                if(nObs>=thresObs)   {
                                    nRedundant++;
                                    break;
                                }
                            }
                    }
                }
            }
        }

        float redudantPerc=float(nRedundant)/float(nPoints);
        if(redudantPerc>System::getParams().KFCulling ){
            KeyFramesThatCanBeRemoved.push_back({redudantPerc,fidx });
        }
    }
    __TSLAM_TIMER_EVENT__("Search done");
    //if more than requested, limit to the best ones
    if ( KeyFramesThatCanBeRemoved.size()>max){
        std::sort(KeyFramesThatCanBeRemoved.begin(),KeyFramesThatCanBeRemoved.end(),[](const pair<float,uint32_t> &a,const pair<float,uint32_t> &b){return a.first>b.first;});
        KeyFramesThatCanBeRemoved.resize(max);
    }
    __TSLAM_TIMER_EVENT__("sort");

    for(auto kf_i:KeyFramesThatCanBeRemoved){
         _debug_msg_("remove frame"<<kf_i.second<<" "<<kf_i.first);
        framesToRemove.insert(kf_i.second);
    }

    return framesToRemove;
}



vector<uint32_t> MapManager::mapPointsCulling(  )
{
    std::vector<uint32_t> pointsToRemove;

    for(auto &mp:TheMap->map_points) {
        if (!mp.isStable() && !mp.isBad()){
            uint32_t obsths=std::min(uint32_t(3),TheMap->keyframes.size());
            if (mp.isStereo())
                obsths=std::min(uint32_t(2),TheMap->keyframes.size());


            if (mp.getVisibility()<0.25 ) mp.setBad (true);
            else if ( mp.kfSinceAddition>=1 && mp.getNumOfObservingFrames()<obsths ) mp.setBad (true);
            else if( mp.kfSinceAddition>=3) mp.setStable (true);
            if ( mp.kfSinceAddition<5) mp.kfSinceAddition++;
        }


        if (mp.isStable())
            if (mp.getVisibility()<0.1 ) mp.setBad (true);
        if(mp.isBad()) pointsToRemove.push_back(mp.id);

    }
    _debug_msg_("mapPointsCulling remove:"<<pointsToRemove.size());
    return pointsToRemove;
}





bool MapManager::mustAddKeyFrame(const Frame & frame_in ,uint32_t curKFRef)
{

    bool reskp=false,resm=false,resrgbd=false;
    if (frame_in.imageParams.isStereoCamera() )
        resrgbd=mustAddKeyFrame_stereo(frame_in,curKFRef );
    if (System::getParams().detectKeyPoints )
        reskp=mustAddKeyFrame_KeyPoints(frame_in,curKFRef );
    if (!reskp && System::getParams().detectMarkers)
        resm= mustAddKeyFrame_Markers(frame_in ,curKFRef);

    //must add the frame info
    return ( reskp || resm||resrgbd );
}

bool MapManager::mustAddKeyFrame_stereo(const Frame &frame_in,uint32_t curKFRef){

    if (!frame_in.imageParams.isStereoCamera()) return false;


    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;

    //count how many close points are seen
    for(size_t i=0;i<frame_in.und_kpts.size();i++){
        if (frame_in.getDepth(i)>0 && frame_in.imageParams.isClosePoint(frame_in.getDepth(i))){
            if (frame_in.ids[i]!=std::numeric_limits<uint32_t>::max() && !frame_in.flags[i].is(Frame::FLAG_OUTLIER))
                nTrackedClose++;
            else
                nNonTrackedClose++;
        }
    }

    _debug_msg_("nTrackedClose="<<nTrackedClose<<" "<<nNonTrackedClose);
     if ((nTrackedClose<180*System::getParams().KFMinConfidence) && (nNonTrackedClose>120*System::getParams().KFMinConfidence))//bNeedToInsertClose
        return true;

     else return false;


}

bool MapManager::mustAddKeyFrame_Markers(const Frame & frame_in ,uint32_t curKFRef){
    // if no marker in the system yet, skip this
    if (TheMap->map_markers.size()==0) return false;
    // if a new marker, not ever seen, add this!
    for(auto m:frame_in.markers){
        if (TheMap->map_markers.count(m.id)==0){
            return true;
        }
    }


    //check if there are invalid markers with unambigous pose in this frame
    for(auto m:frame_in.markers){
        if (TheMap->map_markers.count (m.id)!=0) {//is in the map
            if( TheMap->map_markers.at(m.id).pose_g2m.isValid()==false  ){//with invalid pose
                if ( (frame_in.getMarkerPoseIPPE(m.id).err_ratio>System::getParams().aruco_minerrratio_valid) && System::getParams().aruco_allowOneFrameInitialization){
                    _debug_msg("@@@@@@@@@@@@@@@@ Add keyframe to establish marker pose "<<m.id,10);
                    return true;//adding this will set a valid location for m.id
                }
            }
        }
    }

    //add if the distance to the observation of another markers is far enough
    for(auto m:frame_in.markers){
        if (TheMap->map_markers.count (m.id)!=0) {//is in the map
            const auto &Marker=TheMap->map_markers.at(m.id);
            if( Marker.frames.size()>=System::getParams().maxVisibleFramesPerMarker)continue;
            if( Marker.pose_g2m.isValid() ){//with valid pose
                float minDist=std::numeric_limits<float>::max();
                for( auto f: Marker.frames){
                    float dist= cv::norm(TheMap->keyframes[f].pose_f2g.getTvec(),frame_in.pose_f2g.getTvec());
                    if(dist<minDist)minDist=dist;
                }
                if( minDist>= System::getParams().minBaseLine){
                    _debug_msg(" Add keyframe because baseline ",10);
                     return true;
                }

            }
        }
    }



    //finally, add if baseline with current keyframe is far enough
    //the following only applies if not using keypoints
    if(frame_in.kpts.size()!=0) return false;
    float baseLine=cv::norm(frame_in.pose_f2g.getTvec(),TheMap->keyframes[curKFRef].pose_f2g.getTvec());
    if ( baseLine> System::getParams().minBaseLine){
        _debug_msg(" Add keyframe because baseline ",10);
         return true;
    }

    return false;
}

bool MapManager::mustAddKeyFrame_KeyPoints(const Frame &frame_in, uint32_t curKFRef )
{

    auto getNMatches=[](const Frame &f){
        int n=0;
        for(size_t i=0;i<f.ids.size();i++ )
            if (f.ids[i]!=std::numeric_limits<uint32_t>::max())
                if ( !f.flags[i].is(Frame::FLAG_OUTLIER))
                   n++;
        return n;
    };

    //count how many matches here and in current KFframe
    int mnMatchesInliers=getNMatches(frame_in);
    if (mnMatchesInliers<20)return false;

    float thRefRatio =System::getParams().KFMinConfidence;
    uint32_t minObs=3;

    if(TheMap->keyframes.size()==2){
 //           thRefRatio = 0.4f;
            minObs=2;
    }


    //number of matches of this that are in the keyframe
    int nMatchKF=0;
    const auto &Kframe= TheMap->keyframes[curKFRef];
    for(auto id:Kframe.ids){
        if (id!=std::numeric_limits<uint32_t>::max()){
            const auto &mapP=TheMap->map_points[ id];
            if (mapP.isBad())continue;
            if (mapP.getNumOfObservingFrames()<minObs) continue;
             nMatchKF++;
        }
    }

    _debug_msg_("nMatchesThis="<<mnMatchesInliers<<" nMatchKF="<<nMatchKF<<" thres="<<float(nMatchKF)* thRefRatio);

    if ( mnMatchesInliers<float(nMatchKF)* thRefRatio)return true;
    return false;




}




vector<uint32_t> MapManager::getMatchingFrames(Frame &NewFrame,size_t maxFrames)    {

     //if there is only one frame (plus the new one), use it. We are coming from arucoInitializeFromSingleView
    if (TheMap->keyframes.size()<=2)
        return {TheMap->keyframes.front().idx};

    __TSLAM_ADDTIMER__

    vector<uint32_t> neighbors =TheMap->TheKpGraph.getNeighborsV(NewFrame.idx);//finally, the neighbors
    //remove bad ones
    size_t i=0;
    while( i<neighbors.size()){
        if ( TheMap->keyframes[neighbors[i]].isBad()){
            std::swap(neighbors[i],neighbors.back());
            neighbors.pop_back();
        }
        else i++;
    }

    __TSLAM_TIMER_EVENT__("getNeighborsV");


     std::sort(neighbors.begin(),neighbors.end(),[&](uint32_t a, uint32_t b){return
                TheMap->TheKpGraph.getWeight(a,NewFrame.idx)>TheMap->TheKpGraph.getWeight(b,NewFrame.idx);});

    __TSLAM_TIMER_EVENT__("sort");
    vector<uint32_t> goodframes;
    for(auto neigh: neighbors){
        auto medianDepth=TheMap->getFrameMedianDepth(neigh);
        auto baseline= cv::norm(NewFrame.getCameraCenter()-TheMap->keyframes[neigh].getCameraCenter());
        float acos= NewFrame.getCameraDirection().dot(TheMap->keyframes[neigh].getCameraDirection());
        if(  acos>0.6  &&  baseline/medianDepth> System::getParams().baseline_medianDepth_ratio_min  ) goodframes.push_back(neigh);
        if (goodframes.size()>=maxFrames )break;
    }

    __TSLAM_TIMER_EVENT__("median depth");

    return goodframes;
}


vector<uint32_t> MapManager::searchInNeighbors(Frame &mpCurrentKeyFrame  ){
    auto vpNeighKFs =TheMap->TheKpGraph.getNeighbors (mpCurrentKeyFrame .idx    ); //getMatchingFrames(mpCurrentKeyFrame,20);
    set<uint32_t> vpTargetKFs;
    vector<uint32_t> points2Remove;

    for(auto n: vpNeighKFs)
        if (!TheMap->keyframes[n].isBad())
            vpTargetKFs.insert(n);

    int nAddedObs=0,nFused=0;


    float th=2.5;
    // Search matches by projection from current KF in target KFs
    vector<uint32_t> vpMapPointMatches = mpCurrentKeyFrame.getMapPoints();

      for(auto tkf: vpTargetKFs){
        Frame &keyframe=TheMap->keyframes[tkf];
         cv::Point3f camCenter=keyframe.getCameraCenter();


        for(auto MpId:vpMapPointMatches){
            if ( !TheMap->map_points.is(MpId))continue;
            MapPoint&MP=TheMap->map_points[MpId];
            if (MP.isBad())continue;
            if ( MP.frames.count(keyframe.idx))continue;//already projected
            //project the point
            cv::Point2f p2d= keyframe.project(MP.getCoordinates(),true,true);
            if(isnan(p2d.x))continue;
            float dist= cv::norm(camCenter-MP.getCoordinates());
            if (dist<0.8f*MP.getMinDistanceInvariance() || dist>1.2f*MP.getMaxDistanceInvariance())continue;
            //view angle
            if( MP.getViewCos(camCenter)<0.5)continue;
            //ok, now projection is safe
            int nPredictedLevel = mpCurrentKeyFrame.predictScale(dist,MP.getMaxDistanceInvariance()) ;//MP.predictScale( dist,mLogScaleFactor,keyframe.scaleFactors.size());

            // Search in a radius
              float radius = th*keyframe.scaleFactors[nPredictedLevel];
            if (MP.getViewCos(camCenter)<0.98) radius*=1.4f;
            vector<uint32_t> vkpIdx=keyframe.getKeyPointsInRegion(p2d,radius,nPredictedLevel-1,nPredictedLevel);

            //find the best one
            pair<float,int>  best(System::getParams().maxDescDistance+1e-3,-1);
            for(auto kpidx:vkpIdx){
                float descDist=MP.getDescDistance( keyframe.desc.row(kpidx));
                if ( descDist<best.first)
                    best={descDist,kpidx};
            }

            if (best.second!=-1){
                if ( keyframe.ids[best.second]!=std::numeric_limits<uint32_t>::max()){
                    TheMap->fuseMapPoints(keyframe.ids[best.second],MP.id,false);//the point MP will be partially removed only
                    points2Remove.push_back(MP.id);
                    MP.setBad (true);
                    nFused++;
                }
                else{
                    TheMap->addMapPointObservation(MP.id,keyframe.idx,best.second);
                    nAddedObs++;

                }
            }
        }
    }



    std::vector<uint32_t>  smap_ids=TheMap->getMapPointsInFrames(vpTargetKFs.begin(),vpTargetKFs.end());


    //now, analyze fusion the other way around
    //take the points in the target kframes and project them here
    float mLogScaleFactor=log(mpCurrentKeyFrame.getScaleFactor());
    cv::Point3f camCenter=mpCurrentKeyFrame.getCameraCenter();
    for(auto &mpid:smap_ids){
        auto &MP=TheMap->map_points[mpid];
        if (MP.isBad())continue;
        if(MP.isObservingFrame(mpCurrentKeyFrame.idx)) continue;
        cv::Point2f p2d= mpCurrentKeyFrame.project(MP.getCoordinates(),true,true);
        if(isnan(p2d.x))continue;
        float dist= cv::norm(camCenter-MP.getCoordinates());
        if (dist<0.8f*MP.getMinDistanceInvariance() || dist>1.2f*MP.getMaxDistanceInvariance())continue;
        //view angle
        if( MP.getViewCos(camCenter)<0.5)continue;
        //ok, now projection is safe
        int nPredictedLevel = mpCurrentKeyFrame.predictScale(dist,MP.getMaxDistanceInvariance()); //MP.predictScale( dist,mLogScaleFactor,mpCurrentKeyFrame.scaleFactors.size());

        // Search in a radius
        const float radius = th*mpCurrentKeyFrame.scaleFactors[nPredictedLevel];
        vector<uint32_t> vkpIdx=mpCurrentKeyFrame.getKeyPointsInRegion(p2d,radius,nPredictedLevel-1,nPredictedLevel);
        //find the best one
        pair<float,int>  best(System::getParams().maxDescDistance+1e-3,-1);
        for(auto kpidx:vkpIdx){
            float descDist=MP.getDescDistance( mpCurrentKeyFrame.desc.row(kpidx));
            if ( descDist<best.first)
                best={descDist,kpidx};
        }

        if (best.second!=-1){
            if ( mpCurrentKeyFrame.ids[best.second]!=std::numeric_limits<uint32_t>::max()){
                TheMap->fuseMapPoints(mpCurrentKeyFrame.ids[best.second],MP.id,false);
                points2Remove.push_back(MP.id);
                MP.setBad (true);
                nFused++;
            }
            else{
                TheMap->addMapPointObservation(MP.id,mpCurrentKeyFrame.idx,best.second);
                nAddedObs++;
            }
        }
    }
    _debug_msg_(" searchInNeighbors Added="<<nAddedObs<<" FUSED="<<nFused );

    return  points2Remove;



}



std::list<MapManager::NewPointInfo> MapManager::createCloseStereoPoints(Frame & NewFrame ){
    if (!NewFrame.imageParams.isStereoCamera()) return {};


    //struct to store depth-idx in the frame
    struct kpt_depth_data{
        float depth;
        size_t idx;
        bool operator <(const kpt_depth_data&hp)const{return depth<hp.depth;}
        bool operator >(const kpt_depth_data&hp)const{return depth>hp.depth;}
    };


    if(System::getParams().KPNonMaximaSuppresion)
        NewFrame.nonMaximaSuppresion();
    //count number of close points
    vector<kpt_depth_data> closePoints;
    closePoints.reserve(NewFrame.ids.size());

    for(size_t i=0;i<NewFrame.ids.size();i++)
        if (NewFrame.ids[i]==std::numeric_limits<uint32_t>::max() /*&& NewFrame.und_kpts[i].octave==0*/ && !NewFrame.flags[i].is(Frame::FLAG_NONMAXIMA) && NewFrame.getDepth(i)>0 &&     NewFrame.imageParams.isClosePoint(NewFrame.getDepth(i)) )
            closePoints.push_back({NewFrame.getDepth(i),i});

    if(closePoints.size()>tslam::System::getParams().maxNewPoints){
        std::random_shuffle( closePoints.begin(),closePoints.end());
//        std::sort(closePoints.begin(),closePoints.end(),[](const kpt_depth_data &a,const kpt_depth_data &b){return a.depth<b.depth;});
    }

//    cout<<";???="<<closePoints[0].depth<<" "<<closePoints.back().depth<<" "<< closePoints.size()<<" "<<tslam::System::getParams().maxNewPoints<<endl;exit(0);
    closePoints.resize( std::min( closePoints.size(),size_t(tslam::System::getParams().maxNewPoints)));




//    if(nclose<tslam::System::getParams().maxNewPoints){
//        MinBag<kpt_depth_data> bag;
//        for(size_t i=0;i<NewFrame.ids.size();i++)
//            if (NewFrame.ids[i]==std::numeric_limits<uint32_t>::max() /*&& NewFrame.und_kpts[i].octave==0*/ && NewFrame.getDepth(i)>0  &&!NewFrame.flags[i].is(Frame::FLAG_NONMAXIMA))
//                bag.push({NewFrame.getDepth(i),i});
//        //now, add the points to the usedPoints vector
//        while(!bag.empty() )
//            usedPoints.push_back(bag.pop());
//    }
//    else{
//        for(size_t i=0;i<NewFrame.ids.size();i++)
//            if (NewFrame.ids[i]==std::numeric_limits<uint32_t>::max()/* && NewFrame.und_kpts[i].octave==0*/ && !NewFrame.flags[i].is(Frame::FLAG_NONMAXIMA) &&
//                    NewFrame.getDepth(i)>0 &&    NewFrame.imageParams.isClosePoint(NewFrame.getDepth(i)) )
//                usedPoints.push_back({ NewFrame.getDepth(i),i});
//    }

    std::list<MapManager::NewPointInfo> _NewMapPoints_;
    //get the unassigned points that are close
    auto pose_g2f=NewFrame.pose_f2g.inv();
    for(auto &kpd:closePoints){
        MapManager::NewPointInfo mapPoint;
        //move the point to global coordinates
        mapPoint.pose=pose_g2f*NewFrame.get3dStereoPoint(kpd.idx) ;
        mapPoint.frame_kpt.push_back({ NewFrame.idx,kpd.idx});
        mapPoint.isStereo=true;
        _NewMapPoints_.push_back(mapPoint);;
    }
    //now, find the new points in the other frames


    return _NewMapPoints_;
}

std::vector<MapManager::NewPointInfo>  MapManager::createNewPoints(Frame &NewFrame ,uint32_t nn,uint32_t maxPoints){

    if (NewFrame.ids.size()==0)return{};//no keypoints

    //the set of created matches
    //for each frame, the matches (NewFrame:kptIdx,Frame2:kptIdx)
    struct fmatch{
        fmatch(uint32_t Newframe_kp_idx,uint32_t Frame2_id,uint32_t Frame2_kp_idx,cv::Point3f P3d,float Distance){
            newframe_kp_idx=Newframe_kp_idx;
            frame2_id=Frame2_id;
            frame2_kp_idx=Frame2_kp_idx;
            p3d=P3d;
            distance=Distance;
        }
        uint32_t newframe_kp_idx;
        uint32_t frame2_id;
        uint32_t frame2_kp_idx;
        cv::Point3f p3d;
        float distance;
    };

    //-------------------------------------------------------------------
    // START
    //-------------------------------------------------------------------
    __TSLAM_ADDTIMER__
     Se3Transform TN_G2F=NewFrame.pose_f2g.inv();//matrix moving points from the new frame to the global ref system
    __TSLAM_TIMER_EVENT__("initialize");

    //Create a sorted list of the frames far enough. They are sorted with the number of matched points as sorting order
    vector<uint32_t> matchingFrames =getMatchingFrames(NewFrame,nn);
    __TSLAM_TIMER_EVENT__("compute list of good matching frames");

    //for each possible matching frame, find matches between keypoints
    vector< vector<fmatch> > FrameMatches(matchingFrames.size());//the result is saved here.
    FrameMatcher FMatcher;
    FMatcher.setParams(NewFrame,FrameMatcher::MODE_UNASSIGNED,System::getParams().maxDescDistance*2,0.6,true,std::numeric_limits<int>::max());
//    FrameMatcher_Flann FMatcherFLANN(NewFrame, FrameMatcher_Flann::MODE_UNASSIGNED,System::getParams().minDescDistance*2,0.6,true,std::numeric_limits<int>::max());


    #pragma omp parallel for
    for(int mf=0;mf<int(matchingFrames.size());mf++){
        Frame &frame2=TheMap->keyframes[matchingFrames[mf]];
        //do matching with the newframe
         cv::Mat FQ2T=frame2.pose_f2g*( NewFrame.pose_f2g.inv());
       vector<cv::DMatch> matches=FMatcher.matchEpipolar(frame2,FrameMatcher::MODE_UNASSIGNED,FQ2T);

          vector<cv::Point3f> p3d= Triangulate(NewFrame,frame2,FQ2T,matches);

        ///save the good matches and move the point the frame ref system
        float ratioFactor=1.5f*System::getParams().scaleFactor;
        for(size_t i=0;i<matches.size();i++)
            if (!isnan( p3d[i].x)){
                //check ratioDist
                cv::Point3f p3global=TN_G2F*p3d[i];
                //distance to frames
                float distNF=cv::norm(p3global-NewFrame.getCameraCenter());
                float distF2=cv::norm(p3global-frame2.getCameraCenter());
                if(distNF==0 || distF2==0)continue;
                const float ratioDist = distNF/distF2;
                int oct_NewFrame=   NewFrame.und_kpts[ matches[i].trainIdx].octave;
                int oct_frame2=   frame2.und_kpts[ matches[i].queryIdx].octave;
                const float ratioOctave =NewFrame.scaleFactors[oct_NewFrame] /frame2.scaleFactors[oct_frame2];
                if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                    continue;
                FrameMatches[mf].push_back( fmatch(uint32_t(matches[i].trainIdx),frame2.idx, uint32_t(matches[i].queryIdx),p3global,matches[i].distance));
            }
      //  __TSLAM_TIMER_EVENT__("done with frame");
    }

    auto _to_string=[](const uint32_t&val){ std::stringstream sstr;sstr<<val;return sstr.str();};
    __TSLAM_TIMER_EVENT__("Matches  ("+_to_string(matchingFrames.size())+")");

    //merge all info grouped by keypoint index
    std::map<uint32_t,vector<fmatch> > KptIndex_MatchInfo;
    for(size_t mf=0;mf<FrameMatches.size();mf++)
        for(const auto &match: FrameMatches[mf])
            KptIndex_MatchInfo[ match.newframe_kp_idx].push_back( match);

    std::vector<MapManager::NewPointInfo> _NewMapPoints_;


    for(auto &kp:KptIndex_MatchInfo )
    {

        MapManager::NewPointInfo mapPoint;
        //select the point with lowest descriptor distance
        int bestDesc=-1;
        int bestOctave=std::numeric_limits<int>::max();
        for(size_t di=0;di<kp.second.size();di++){
            const auto &frame_kp= TheMap->keyframes[ kp.second[di].frame2_id].und_kpts[ kp.second[di].frame2_kp_idx];
            if(  frame_kp.octave<bestOctave){bestDesc=di;}
 //           if ( kp.second[bestDesc].distance<kp.second[di].distance) bestDesc=di;
        }

        const auto &best_match=kp.second[bestDesc];

        mapPoint.pose= best_match.p3d;
        mapPoint.dist=best_match.distance;
        mapPoint.frame_kpt.push_back(  {NewFrame.idx, kp.first });
        //now, add all
       // mapPoint.frame_kpt.push_back(  {TheMap->keyframes[best_match.frame2_id].idx, best_match.frame2_kp_idx });

        for(auto fma:kp.second)
            mapPoint.frame_kpt.push_back(  {TheMap->keyframes[fma.frame2_id].idx, fma.frame2_kp_idx });
        _NewMapPoints_.push_back(mapPoint);;
    }


    if (_NewMapPoints_.size()>maxPoints){
        std::sort(_NewMapPoints_.begin(),_NewMapPoints_.end(),[](const MapManager::NewPointInfo &a,const MapManager::NewPointInfo &b){return a.dist<b.dist;});
        _NewMapPoints_.resize(maxPoints);
    }
    __TSLAM_TIMER_EVENT__("Merging and creation");
    _debug_msg_("Added "<<_NewMapPoints_.size()<<" new points");



    return _NewMapPoints_;

}

void MapManager:: globalOptimization(int niters ){
     GlobalOptimizer::ParamSet params( debug::Debug::getLevel()>=11);
    //must set the first and second as fixed?
    params.fixFirstFrame=true;
    params.nIters=niters;
    params.markersOptWeight=System::getParams().markersOptWeight;
    params.minMarkersForMaxWeight=System::getParams().minMarkersForMaxWeight;
    params.InPlaneMarkers=System::getParams().inPlaneMarkers;


    if (params.fixed_frames.size()==0 && TheMap->map_markers.size()==0){//fixed second one to keep the scale if no markers
        //get the second frame if there is one and set it fixed
        auto it=TheMap->keyframes.begin();
        params.fixed_frames.insert(it->idx);
        ++it;
        if (it!=TheMap->keyframes.end()) {
            if(params.used_frames.count(it->idx) || params.used_frames.size()==0 )
                params.fixed_frames.insert(it->idx);
        }
    }

    Gopt=GlobalOptimizer::create(System::getParams().global_optimizer);
    Gopt->setParams(TheMap,params);
    Gopt->optimize();
    Gopt->getResults(TheMap);
    TheMap->removeBadAssociations(Gopt->getBadAssociations(),System::getParams().minNumProjPoints);
    Gopt=nullptr;
}



void MapManager:: localOptimization(uint32_t _newKFId,int nIters){
      //add current frame and all the connected to them for optimization
    //then, also add as fixed all these in which elements in former frames project

    //is there any stereo
    bool hasStereo=false;
    for(auto f:TheMap->keyframes)
        if (f.imageParams.isStereoCamera()){
            hasStereo=true;
            break;
        }

    std::set<uint32_t> neigh=     TheMap->TheKpGraph.getNeighbors(_newKFId,true);

     GlobalOptimizer::ParamSet params(  debug::Debug::getLevel()>=11 );
     params.markersOptWeight=System::getParams().markersOptWeight;
     params.minMarkersForMaxWeight=System::getParams().minMarkersForMaxWeight;
     params.used_frames.insert(neigh.begin(),neigh.end());
    //must set the first and second as fixed?
    params.fixFirstFrame=true;
    params.nIters=nIters;
    params.InPlaneMarkers=System::getParams().inPlaneMarkers;


    if (params.fixed_frames.size()==0 && TheMap->map_markers.size()==0 && !hasStereo){//fixed second one to keep the scale if no markers
        //get the second frame if there is one and set it fixed
        auto it=TheMap->keyframes.begin();
        params.fixed_frames.insert(it->idx);
        ++it;
        if (it!=TheMap->keyframes.end()) {
            if(params.used_frames.count(it->idx))
                params.fixed_frames.insert(it->idx);
        }
    }

    Gopt=GlobalOptimizer::create(System::getParams().global_optimizer);
    Gopt->setParams(TheMap,params);
     Gopt->optimize(&_hurryUp);
 }

void MapManager::toStream(std::ostream &str) {
    //must wait until no processing is being done

    while(_curState==WORKING) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //now, in case in state previous to store results, save them and then continue
     mapUpdate();

    uint64_t sig=1823312417;
    str.write((char*)&sig,sizeof(sig));
    str.write((char*)&_lastAddedKeyFrame,sizeof(_lastAddedKeyFrame));
    str.write((char*)&nFramesAnalyzedWithoutAddingKF,sizeof(nFramesAnalyzedWithoutAddingKF));



    str.write((char*)&mustExit,sizeof(mustExit));
    auto val=_curState.load();
    str.write((char*)&val,sizeof(val));
    toStream__(keyframesToAdd.buffer_,str);
    toStream__(PointsToRemove,str);
    toStream__(KeyFramesToRemove,str);
    toStream__kv(youngKeyFrames,str);
    str.write((char*)&_hasMapBeenScaled,sizeof(_hasMapBeenScaled));
    _lastAddedKFPose.toStream(str);
    str.write((char*)&bigChangeHasHappen,sizeof(bigChangeHasHappen));
    str.write((char*)&_CurkeyFrame,sizeof(_CurkeyFrame));
    str.write((char*)&_mustAddKeyFrame,sizeof(_mustAddKeyFrame));
    str.write((char*)&_hurryUp,sizeof(_hurryUp));

    _LoopClosureInfo.toStream(str);


}
void MapManager::fromStream(std::istream &str){

    stop();

    uint64_t sig;
    str.read((char*)&sig,sizeof(sig));
    if(sig!=1823312417) throw std::runtime_error(string(__PRETTY_FUNCTION__)+"Could not read signature of Mapmanager in stream");
    str.read((char*)&_lastAddedKeyFrame,sizeof(_lastAddedKeyFrame));
    str.read((char*)&nFramesAnalyzedWithoutAddingKF,sizeof(nFramesAnalyzedWithoutAddingKF));

    str.read((char*)&mustExit,sizeof(mustExit));
    auto cstatevar=_curState.load();
    str.read((char*)&cstatevar,sizeof(cstatevar));
    _curState=cstatevar;

    fromStream__(keyframesToAdd.buffer_,str);
    fromStream__(PointsToRemove,str);
    fromStream__(KeyFramesToRemove,str);
    fromStream__kv(youngKeyFrames,str);
    str.read((char*)&_hasMapBeenScaled,sizeof(_hasMapBeenScaled));
    _lastAddedKFPose.fromStream(str);
    str.read((char*)&bigChangeHasHappen,sizeof(bigChangeHasHappen));
    str.read((char*)&_CurkeyFrame,sizeof(_CurkeyFrame));
    str.read((char*)&_mustAddKeyFrame,sizeof(_mustAddKeyFrame));
    str.read((char*)&_hurryUp,sizeof(_hurryUp));
    _LoopClosureInfo.fromStream(str);



}

uint64_t MapManager::getSignature(){

    Hash sig;
    sig+=_lastAddedKeyFrame;
   // cout<<"mm1. sig="<<sig<<endl;
    sig+=nFramesAnalyzedWithoutAddingKF;
   // cout<<"mm2. sig="<<sig<<endl;
    sig+=mustExit;
   // cout<<"mm3. sig="<<sig<<endl;
    sig+=_curState.load();
   // cout<<"mm4. sig="<<sig<<endl;
    sig+=keyframesToAdd.size();
   // cout<<"mm5. sig="<<sig<<endl;
    for(auto kv:PointsToRemove) sig+=kv;
   // cout<<"mm6. sig="<<sig<<endl;
    for(auto kv:KeyFramesToRemove) sig+=kv;
   // cout<<"mm7. sig="<<sig<<endl;
    for(auto kv:youngKeyFrames) {sig+=kv.first;sig+=kv.second;}
   // cout<<"mm8. sig="<<sig<<endl;
    sig+=_hasMapBeenScaled;
   // cout<<"mm9. sig="<<sig<<endl;
    sig+=_lastAddedKFPose;
   // cout<<"mm10. sig="<<sig<<endl;
    sig+=bigChangeHasHappen;
   // cout<<"mm11. sig="<<sig<<endl;
    sig+=_CurkeyFrame;
   // cout<<"mm12. sig="<<sig<<endl;
    sig+=_mustAddKeyFrame;
   // cout<<"mm13. sig="<<sig<<endl;
    sig+=_LoopClosureInfo.getSignature();
   // cout<<"mm14. sig="<<sig<<endl;
    return sig;
}

//adds the keyframe and correct and join points from both sides
void MapManager::loopClosurePostProcessing(Frame &inFrame, const LoopDetector::LoopClosureInfo &lci){
    auto toSet=[](const vector<uint32_t> &v){
        std::set<uint32_t> s;
        for(auto e:v) s.insert(e);
        return s;
    };

    auto neighborsSideOld=TheMap->TheKpGraph.getNeighborsV(lci.matchingFrameIdx,true);
    auto neighborsSideNew=TheMap->TheKpGraph.getNeighborsV( lci.curRefFrame,true);

    //remove possible connection with other side of the loop

//    for(auto &id:frame.ids){
//        if (id==std::numeric_limits<uint32_t>::max())continue;
//        for(auto n:neighborsSideOld)
//            if (TheMap->map_points[id].isObservingFrame(n))
//                TheMap->removeMapPointObservation( id,frame.idx,System::getParams().minNumProjPoints);
//        }

    //check that both loop sides are not yet connected
//    for(auto n:neighborsSideOld)
//        for(auto nn:neighborsSideNew){
//            assert( !TheMap->TheKpGraph.isEdge(n,nn));
//        }


    auto &NewFrame=inFrame;
    if (!TheMap->keyframes.is(inFrame.idx)){//add the keyframe if not yet (in case of marker loopclosure)
            NewFrame.pose_f2g=lci.expectedPos;
           NewFrame= addKeyFrame(&inFrame);
    }

    //add to the frame, the points seen in the other side of the loop
    int nMOb=0;
    for(auto match:lci.map_matches){
        if (NewFrame.ids[match.queryIdx]==std::numeric_limits<uint32_t>::max() && !TheMap->map_points[match.trainIdx].isObservingFrame(NewFrame.idx)){
            TheMap->addMapPointObservation(match.trainIdx,NewFrame.idx,match.queryIdx);
             nMOb++;
        }
    }

    globalOptimization(20);

    auto allNeigh=TheMap->TheKpGraph.getNeighborsV(NewFrame.idx,true);

    vector<float> distances={4,2.5};
    for(size_t nt=0;nt<distances.size();nt++){
        nMOb=0;
        int nFusions=0;
        for(auto fidx:allNeigh){
            auto &CurFrame=TheMap->keyframes[fidx];
                //now, look for map points
                auto frameMapPoints=toSet(CurFrame.getMapPoints());
                auto map_matches =TheMap->matchFrameToMapPoints(allNeigh, CurFrame,
                                                                 CurFrame.pose_f2g,System::getParams().maxDescDistance*2, distances[nt],
                                                                 false,true,frameMapPoints);

                for(auto match:map_matches){
                    assert( frameMapPoints.count(match.trainIdx)==0);

                    assert( !TheMap->map_points[match.trainIdx].isObservingFrame(CurFrame.idx));
                    //if the keypoint is already assigned, fuse it
                    if(  CurFrame.ids[match.queryIdx]!=std::numeric_limits<uint32_t>::max()){
                        TheMap->fuseMapPoints(match.trainIdx,CurFrame.ids[match.queryIdx],true);
                        nFusions++;
                    }
                    else{
                        TheMap->addMapPointObservation(match.trainIdx,CurFrame.idx,match.queryIdx);
                        nMOb++;
                    }
            }
        }
         if (nMOb>0 || nFusions>0){
            localOptimization(NewFrame.idx,20);
            TheMap->removeBadAssociations(Gopt->getBadAssociations(),System::getParams().minNumProjPoints);
        }

    }

    inFrame.pose_f2g=NewFrame.pose_f2g;
    inFrame.ids=NewFrame.ids;


}



}

