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
#include <list>
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <aruco/markermap.h>
#include "system.h"
#include "basictypes/misc.h"
#include "basictypes/debug.h"
#include "basictypes/timers.h"
#include "optimization/pnpsolver.h"
#include "optimization/globaloptimizer.h"
#include "optimization/ippe.h"
#include "basictypes/io_utils.h"
#include "map_types/keyframedatabase.h"
#include "utils/mapinitializer.h"
#include "utils/mapmanager.h"
#include "map.h"
#include "basictypes/se3.h"
#include "basictypes/osadapter.h"
#include "map_types/covisgraph.h"
#include "utils/frameextractor.h"
#include "basictypes/hash.h"

namespace tslam{
   //System global params
   Params System::_params;
   Params  & System::getParams()  {return _params;}



   //returns the index of the current keyframe
   uint32_t System::getCurrentKeyFrameIndex(){return _curKFRef;}


   //returns a pointer to the map being used
   std::shared_ptr<Map> System::getMap(){return TheMap;}


System::System(){
     fextractor=std::make_shared<FrameExtractor>();
     map_initializer=std::make_shared<MapInitializer>();
     TheMapManager=std::make_shared<MapManager>();
     marker_detector=std::make_shared<tslam::STagMarkerDetector>();
}

System::~System(){

    waitForFinished();
}


void System::createFrameExtractor(){
    _params.nthreads_feature_detector=max(1,_params.nthreads_feature_detector);
    std::shared_ptr<Feature2DSerializable> fdetector=Feature2DSerializable::create(_params.kpDescriptorType );

    fdetector->setParams(_params.extraParams);
    _params.maxDescDistance=fdetector->getMinDescDistance();


    fextractor->setParams(fdetector, _params,marker_detector);
    fextractor->removeFromMarkers()=_params.removeKeyPointsIntoMarkers;
    fextractor->detectMarkers()=_params.detectMarkers;
    fextractor->detectKeyPoints()=_params.detectKeyPoints;

}
void System::setParams( std::shared_ptr<Map> map, const  Params &p,const string &vocabulary,std::shared_ptr<tslam::MarkerDetector> mdetector){

    TheMap=map;


    _params=p;
    marker_detector=mdetector;
    if(!marker_detector)
        marker_detector=std::make_shared<STagMarkerDetector>(_params);
    createFrameExtractor();

    //now, the vocabulary

    if (TheMap->isEmpty()){//Need to start from zero
        currentState=STATE_LOST;
        if (!vocabulary.empty()  ){
            TheMap->TheKFDataBase.loadFromFile(vocabulary);
        }
        MapInitializer::Params params;
        if ( _params.forceInitializationFromMarkers)
            params.mode=MapInitializer::ARUCO;
        else
            params.mode=MapInitializer::BOTH;

        params.minDistance= _params.minBaseLine;
        params.markerSize=_params.aruco_markerSize;
        params.aruco_minerrratio_valid= _params.aruco_minerrratio_valid;
        params.allowArucoOneFrame=_params.aruco_allowOneFrameInitialization;
        params.max_makr_rep_err=2.5;
        params.minDescDistance=_params.maxDescDistance;
        map_initializer->setParams(params);
    }
    else
        currentState=STATE_LOST;
}

void System::waitForFinished(){
 TheMapManager->stop();
 TheMapManager->mapUpdate();
 if(TheMapManager->bigChange()){
     _cFrame.pose_f2g=TheMapManager->getLastAddedKFPose();
     _curPose_f2g=_cFrame.pose_f2g;
 }
}
//void System::resetCurrentPose(){
//    waitForFinished();

//    if (currentState==STATE_TRACKING)
//        currentState=STATE_LOST;
//    _curPose_f2g=cv::Mat();
//    velocity=cv::Mat();
//}
void System::resetTracker(){
    waitForFinished();
    _curKFRef=-1;
    _curPose_f2g=se3();
    currentState=STATE_LOST;
    _cFrame.clear();
    _prevFrame.clear();
    velocity=cv::Mat();
    lastKFReloc=-1;
}


cv::Mat System::process(const Frame &frame) {
     assert(TheMap->checkConsistency());
     se3 prevPose=_curPose_f2g;//pose computed

    //copy the current frame if not calling from the other member funtion
    if ((void*)&frame!=(void*)&_cFrame){//only if not calling from the other process member function
        swap(_prevFrame,_cFrame);
        _cFrame=frame;
    }


    _debug_exec(20, saveToFile("world-prev.ucs"););

    __TSLAM_ADDTIMER__;

    //Initialize other processes if not yet
    if (currentMode==MODE_SLAM  && !TheMapManager->hasMap())
        TheMapManager->setParams(TheMap,_params.enableLoopClosure);
    if (!_params.runSequential && currentMode==MODE_SLAM )
        TheMapManager->start();

    __TSLAM_TIMER_EVENT__  ("initialization");
    //update map if required


    //remove possible references to removed mappoints in _prevFrame
    for(auto &id:_prevFrame.ids)
        if (id!=std::numeric_limits<uint32_t>::max()){
            if (!TheMap->map_points.is(id)) id=std::numeric_limits<uint32_t>::max();
            else if( TheMap->map_points[id].isBad())id=std::numeric_limits<uint32_t>::max();
        }
    __TSLAM_TIMER_EVENT__("removed invalid map references");


    _debug_msg_("\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" );
    _debug_msg_("|||||||||||||||||||||| frame="<<frame.fseq_idx<<" sig="<<sigtostring(frame.getSignature())<<"  Wsig="<<sigtostring(getSignature()));
    _debug_msg_("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" );



    //not initialized yet
    if(TheMap->isEmpty() && currentMode==MODE_SLAM) {
        if ( initialize(_cFrame))
            currentState=STATE_TRACKING;
        __TSLAM_TIMER_EVENT__("initialization attempted");
    }
    else{
        //tracking mode
        if( currentState==STATE_TRACKING){
            _curKFRef=getBestReferenceFrame(_prevFrame,_curPose_f2g);
            _curPose_f2g=track(_cFrame,_curPose_f2g);
            _debug_msg_("current pose="<<_curPose_f2g);
            __TSLAM_TIMER_EVENT__("track");
            if( !_curPose_f2g.isValid())
                currentState=STATE_LOST;
        }
        //lost??
        if (currentState==STATE_LOST){
            se3 reloc_pose;
            if ( relocalize(_cFrame,reloc_pose)){//recovered
                currentState=STATE_TRACKING;
                _curPose_f2g=reloc_pose;
                _curKFRef=getBestReferenceFrame(_cFrame,_curPose_f2g);
                lastKFReloc=_cFrame.fseq_idx;
            }
            __TSLAM_TIMER_EVENT__("relocalize");
        }


        //did properly recover??
        if( currentState==STATE_TRACKING){
            _cFrame.pose_f2g=_curPose_f2g;
            if (currentMode==MODE_SLAM  && (  ( _cFrame.fseq_idx>=lastKFReloc+5) || (lastKFReloc==-1)  ))//must add a new frame?
               TheMapManager->newFrame(_cFrame,_curKFRef);
            __TSLAM_TIMER_EVENT__("newFrame");
            _debug_msg_("_curKFRef="<<_curKFRef);
        }
    }

    __TSLAM_TIMER_EVENT__("track/initialization done");

    assert(TheMap->checkConsistency(debug::Debug::getLevel()>=10));


    //if lost after very few keyframes, reset map
    if( currentState==STATE_LOST && currentMode==MODE_SLAM && TheMap->keyframes.size()<=5 && TheMap->keyframes.size()!=0){
        TheMapManager->reset();
        TheMap->clear();
        map_initializer->reset();
        TheMapManager->setParams(TheMap,_params.enableLoopClosure);
    }


    //compute the velocity
     if (currentState==STATE_TRACKING ){
        velocity=cv::Mat::eye(4,4,CV_32F);
        if ( prevPose.isValid()){
            velocity = _curPose_f2g.convert()*prevPose.convert().inv();
         }
    }
    else{
        velocity=cv::Mat();

    }


    _cFrame.pose_f2g=_curPose_f2g;
    _debug_msg_("camera pose="<<_curPose_f2g);

//#ifndef _TSLAM_123asd
//    if( ++totalNFramesProcessed> (10*4*12*34*6)/2)
//        _curPose_f2g=cv::Mat();
//#endif

    if (currentState==STATE_LOST) return cv::Mat();
    else return _curPose_f2g;

}



cv::Mat System::process(  cv::Mat &InputImage, const ImageParams &img_params,uint32_t frameseq_idx, const cv::Mat & depth, const cv::Mat &RIn_image) {



    __TSLAM_ADDTIMER__;

    assert( (img_params.bl>0 && !depth.empty()) || depth.empty());//if rgbd, then ip must  have the params set

    swap(_prevFrame,_cFrame);

    __TSLAM_TIMER_EVENT__("Input preparation");
    //update the map while processing this new frame to extract keypoints an markers
    std::thread UpdateThread;
    if (currentMode==MODE_SLAM) UpdateThread=std::thread([&](){
        if( TheMapManager->mapUpdate()){
            if(TheMapManager->bigChange()){
                _cFrame.pose_f2g= TheMapManager->getLastAddedKFPose();
                _curPose_f2g=TheMapManager->getLastAddedKFPose();
            }
        }
        ;});
    //determine the internal id that will be given to this frame
    if (depth.empty() && RIn_image.empty()) fextractor->process(InputImage,img_params,_cFrame,frameseq_idx   );//c
    else if(RIn_image.empty()) fextractor->process_rgbd(InputImage,depth,img_params,_cFrame,frameseq_idx );
    else fextractor->processStereo(InputImage,RIn_image,img_params,_cFrame,frameseq_idx );



    if( _params.autoAdjustKpSensitivity){
        //automatically adjust sensitivity
        //if few keypoints, may be the scene is dark and needs increasing value
        //auto adjust sensitivity by checking if it fits the desired number of keypoints
        int missingKpts= _params.maxFeatures-_cFrame.und_kpts.size();
        if(  missingKpts >0){
            float perct=1.0f- (float(missingKpts)/float(_cFrame.und_kpts.size()));
            float newSensitivity=fextractor->getSensitivity()+perct;
            newSensitivity=std::max(newSensitivity,1.0f);
            fextractor->setSensitivity( newSensitivity);
        }
        else{//the number of points is correct
            //if sensitivity is not at minimum, decrease it a little bit
            fextractor->setSensitivity(fextractor->getSensitivity()*0.95);
        }
        _debug_msg_("KptDetector Sensitivity="<<fextractor->getSensitivity());
    }
    //
    __TSLAM_TIMER_EVENT__("frame extracted ");
    if (currentMode==MODE_SLAM)UpdateThread.join(); //wait for update


    cv::Mat result=process(_cFrame);
    //draw matches in image
    __TSLAM_TIMER_EVENT__("process");

    float ImageScaleFactor= sqrt(float(_cFrame.imageParams.CamSize.area())/float(InputImage.size().area()));

    drawMatchesAndMarkersInInputImage(InputImage,1./ImageScaleFactor);


    auto _to_string=[](const uint32_t&val){ std::stringstream sstr;sstr<<val;return sstr.str();};

    putText(InputImage,"Map Points:"+_to_string(TheMap->map_points.size()),cv::Point(20,InputImage.rows-20));
    putText(InputImage,"Map Markers:"+_to_string(TheMap->map_markers.size()),cv::Point(20,InputImage.rows-40));
    putText(InputImage,"KeyFrames:"+_to_string(TheMap->keyframes.size()),cv::Point(20,InputImage.rows-60));
    int nmatches=0;
    for(auto id:_cFrame.ids) if(id!=std::numeric_limits<uint32_t>::max()) nmatches++;
    putText(InputImage,"Matches:"+  _to_string(nmatches),cv::Point(20,InputImage.rows-80));
    if( fabs(ImageScaleFactor-1)>1e-3)
        putText(InputImage,"Img.Size:"+_to_string(_cFrame.imageParams.CamSize.width)+"x"+_to_string(_cFrame.imageParams.CamSize.height),cv::Point(20,InputImage.rows-100));


    __TSLAM_TIMER_EVENT__("draw");

    return result;

}


void  System::putText(cv::Mat &im,string text,cv::Point p ){
    float fact=float(im.cols)/float(1280);

    cv::putText(im,text,p,cv::FONT_HERSHEY_SIMPLEX, 0.5*fact,cv::Scalar(0,0,0),3*fact);
    cv::putText(im,text,p,cv::FONT_HERSHEY_SIMPLEX, 0.5*fact,cv::Scalar(125,255,255),1*fact);

}
string System::getSignatureStr()const{
    return sigtostring(getSignature());
}

uint64_t System::getSignature(bool print)const{

    Hash sig;
    sig+=TheMap->getSignature(print);
    if(print)cout<<"\tSystem 1. sig="<<sig<<endl;
    sig+=_params.getSignature();
    if(print)cout<<"\tSystem 2. sig="<<sig<<endl;
    for(int i=0;i<6;i++)sig+=_curPose_f2g[i];
    if(print)cout<<"\tSystem 3. sig="<<sig<<endl;
    sig.add(_curKFRef);
    if(print)cout<<"\tSystem 4. sig="<<sig<<endl;
    sig+=_cFrame.getSignature();
    if(print)cout<<"\tSystem 5. sig="<<sig<<endl;

    sig+=isInitialized;
    if(print)cout<<"\tSystem 7. sig="<<sig<<endl;
    sig+=currentState;
    if(print)cout<<"\tSystem 8. sig="<<sig<<endl;
    sig+=currentMode;
    if(print)cout<<"\tSystem 9. sig="<<sig<<endl;
    sig+=_prevFrame.getSignature();
    if(print)cout<<"\tSystem 10.sig="<<sig<<endl;
    sig+=TheMapManager->getSignature();
    if(print)cout<<"\tSystem 11.sig="<<sig<<endl;
    sig+=velocity;
    sig+=lastKFReloc;
    if(print)cout<<"\tSystem 12.sig="<<sig<<endl;
    return sig;


}





//given a frame and a map, returns the set pose using the markers
//If not possible, return empty matrix
//the pose matrix returned is from Global 2 Frame
cv::Mat System::getPoseFromMarkersInMap(const Frame &frame ){
    std::vector<uint32_t> validmarkers;//detected markers that are in the map

    //for each marker compute the set of poses
    vector<pair<cv::Mat,double> > pose_error;
    vector<cv::Point3f> markers_p3d;
    vector<cv::Point2f> markers_p2d;

    for(auto m:frame.markers){
        if (TheMap->map_markers.find(m.id)!=TheMap->map_markers.end()){
            tslam::Marker &mmarker=TheMap->map_markers[m.id];
            cv::Mat Mm_pose_g2m=mmarker.pose_g2m;
            //add the 3d points of the marker
            auto p3d=mmarker.get3DPoints();
            markers_p3d.insert(markers_p3d.end(),p3d.begin(),p3d.end());
            //and now its 2d projection
            markers_p2d.insert(markers_p2d.end(),m.und_corners.begin(),m.und_corners.end());

            auto poses_f2m=IPPE::solvePnP(_params.aruco_markerSize,m.und_corners,frame.imageParams.CameraMatrix,frame.imageParams.Distorsion);
            for(auto pose_f2m:poses_f2m)
                pose_error.push_back(   make_pair(pose_f2m * Mm_pose_g2m.inv(),-1));
        }
    }
    if (markers_p3d.size()==0)return cv::Mat();
    //now, check the reprojection error of each solution in all valid markers and take the best one
    for(auto &p_f2g:pose_error){
        vector<cv::Point2f> p2d_reprj;
        se3 pose_se3=p_f2g.first;
        project(markers_p3d,frame.imageParams.CameraMatrix,pose_se3.convert(),p2d_reprj);
//        cv::projectPoints(markers_p3d,pose_se3.getRvec(),pose_se3.getTvec(),TheImageParams.CameraMatrix,TheImageParams.Distorsion,p2d_reprj);
        p_f2g.second=0;
        for(size_t i=0;i<p2d_reprj.size();i++)
            p_f2g.second+= (p2d_reprj[i].x- markers_p2d[i].x)* (p2d_reprj[i].x- markers_p2d[i].x)+ (p2d_reprj[i].y- markers_p2d[i].y)* (p2d_reprj[i].y- markers_p2d[i].y);
    }

    //sort by error

    std::sort(pose_error.begin(),pose_error.end(),[](const pair<cv::Mat,double> &a,const pair<cv::Mat,double> &b){return a.second<b.second; });
    //    for(auto p:pose_error)
    //    cout<<"p:"<<p.first<<" "<<p.second<<endl;
    return pose_error[0].first;//return the one that minimizes the error
}



bool System::initialize( Frame &f2 ) {
bool res;
    if (f2.imageParams.isStereoCamera()){
          res=initialize_stereo(f2);
    }
    else{
        res=initialize_monocular(f2);
    }

    if(!res)return res;
    _curPose_f2g= TheMap->keyframes.back().pose_f2g;
    _curKFRef=TheMap->keyframes.back().idx;
    _debug_msg_("Initialized");
    isInitialized=true;
    assert(TheMap->checkConsistency(true));

  return true;
}

bool System::initialize_monocular(Frame &f2 ){

    _debug_msg_("initialize   "<<f2.markers.size());

    if (!map_initializer->process(f2,TheMap)) return false;

      //If there keypoints  and at least 2 frames
    // set the ids of the visible elements in f2, which will used in tracking
        if ( TheMap->keyframes.size()>1 && TheMap->map_points.size()>0){
        f2.ids = TheMap->keyframes.back().ids;
    }
    assert(TheMap->checkConsistency());


    globalOptimization();
     assert(TheMap->checkConsistency(true));
    //if only with matches, scale to have an appropriate mean
    if (TheMap->map_markers.size()==0){
        if ( TheMap->map_points.size()<50) {//enough points after the global optimization??
            TheMap->clear();
            return false;
        }


        float invMedianDepth=1./TheMap->getFrameMedianDepth(TheMap->keyframes.front().idx);
        cv::Mat Tc2w=TheMap->keyframes.back().pose_f2g.inv();


        //change the translation
        // Scale initial baseline
        Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
        TheMap->keyframes.back().pose_f2g=Tc2w.inv();

         // Scale points
         for(auto &mp:TheMap->map_points){
            mp.scalePoint(invMedianDepth);
        }

    }

    _curPose_f2g=TheMap->keyframes.back().pose_f2g;

      return true;
}



bool System::initialize_stereo( Frame &frame){
    __TSLAM_ADDTIMER__;

    if(_params.KPNonMaximaSuppresion)
        frame.nonMaximaSuppresion();
    int nValidPoints=0;
    for(size_t i=0;i<frame.und_kpts.size();i++){
        //check the stereo depth
        if (frame.getDepth(i)>0 && frame.imageParams.isClosePoint(frame.getDepth(i)) &&  !frame.flags[i].is(Frame::FLAG_NONMAXIMA))
            nValidPoints++;
    }

    if ( nValidPoints<100) return false;


    frame.pose_f2g.setUnity();
    Frame & kfframe1=TheMap->addKeyFrame(frame); //[frame2.idx]=frame2;

    //now, get the 3d point information from the depthmap
    for(size_t i=0;i<frame.und_kpts.size();i++){
        //check the stereo depth
        cv::Point3f p ;
        if ( frame.getDepth(i)>0 && frame.imageParams.isClosePoint(frame.getDepth(i)) && !frame.flags[i].is(Frame::FLAG_NONMAXIMA)){
            //compute xyz
            p=frame.get3dStereoPoint(i);
            auto &mp= TheMap->addNewPoint(kfframe1.fseq_idx);
            assert(!mp.isStable());
            mp.kfSinceAddition=1;
            mp.setCoordinates(p);
            mp.setStereo(true);
            TheMap->addMapPointObservation(mp.id,kfframe1.idx,i);
            frame.ids[i]=mp.id;
        }
    }
    //now, add the markers
    for(const auto &m:frame.markers ){
        TheMap-> addMarker( m );
    }
    return true;
}

string System::sigtostring(uint64_t sig)const{
    string sret;
    string alpha="qwertyuiopasdfghjklzxcvbnm1234567890QWERTYUIOPASDFGHJKLZXCVBNM";
    uchar * s=(uchar *)&sig;
    int n=sizeof(sig)/sizeof(uchar );
    for(int i=0;i<n;i++){
        sret.push_back(alpha[s[i]%alpha.size()]);
    }
    return sret;
}





uint32_t System::getBestReferenceFrame(const Frame &curKeyFrame,  const se3 &curPose_f2g){

    __TSLAM_ADDTIMER__
    //    if( map_matches.size()==0 && TheMap->map_markers.size()==0)return _curKFRef;

    int64_t bestCandidateKeyPoints=-1 ;

    if (_params.detectKeyPoints)
       bestCandidateKeyPoints=TheMap->getReferenceKeyFrame(curKeyFrame,1);
    if (bestCandidateKeyPoints!=-1) return bestCandidateKeyPoints;


    //try with the markers now
    if ( TheMap->map_markers.size()==0){//NO MARKERS!!
        assert(_curKFRef!=-1);
        return _curKFRef;
    }
    //determine all valid markers seen here
    vector<uint32_t> validMarkers;
    for(auto m:curKeyFrame.markers){
        auto marker=  TheMap->map_markers.find(m.id);
        if ( marker!=TheMap->map_markers.end()){
            if (marker->second.pose_g2m.isValid()) validMarkers.push_back(m.id);
        }
    }

    pair<uint32_t,float> nearest_frame_dist(std::numeric_limits<uint32_t>::max(),std::numeric_limits<float>::max());
    for(auto marker:validMarkers)
        for(const auto &frame:TheMap->map_markers[marker].frames){
            assert(TheMap->keyframes.is(frame));
            auto d=   curPose_f2g .t_dist( TheMap->keyframes[frame].pose_f2g )     ;
            if ( nearest_frame_dist.second>d) nearest_frame_dist={frame,d};
        }
    return nearest_frame_dist.first;


}
std::vector<System::kp_reloc_solution> System::relocalization_withkeypoints_( Frame &curFrame,se3 &pose_f2g_out ,const std::set<uint32_t> &excluded ){
    _debug_msg_("");
    if (curFrame.ids.size()==0)return { };
    if (TheMap->TheKFDataBase.isEmpty())return  {};
    vector<uint32_t> kfcandidates=TheMap->relocalizationCandidates(curFrame,excluded);

    if (kfcandidates.size()==0)return  {};

    vector<System::kp_reloc_solution> Solutions(kfcandidates.size());

    FrameMatcher FMatcher;
    FMatcher.setParams(curFrame,FrameMatcher::MODE_ALL,_params.maxDescDistance*2);

#pragma omp parallel for
    for(int cf=0;cf<kfcandidates.size();cf++){

        auto kf=kfcandidates[cf];
        auto &KFrame=TheMap->keyframes[kf];
        Solutions[cf].matches=FMatcher.match(KFrame,FrameMatcher::MODE_ASSIGNED);


        //change trainIdx and queryIdx to match the  solvePnpRansac requeriments
        for(auto &m:Solutions[cf].matches){
            std::swap(m.queryIdx,m.trainIdx);
            m.trainIdx= KFrame.ids[m.trainIdx];
        }
        //remove bad point matches
        for(int i=0;i<Solutions[cf].matches.size();i++){
            auto &mp=Solutions[cf].matches[i].trainIdx;
            if( !TheMap->map_points.is(mp))
                Solutions[cf].matches[i].trainIdx=-1;
            if( TheMap->map_points[mp].isBad()  )
                Solutions[cf].matches[i].trainIdx=-1;
        }

        remove_unused_matches(Solutions[cf].matches);
        if (Solutions[cf].matches.size()<25)continue;
        Solutions[cf].pose=KFrame.pose_f2g;
         //estimate initial position
        PnPSolver::solvePnPRansac(curFrame,TheMap,Solutions[cf].matches,Solutions[cf].pose);
         if (Solutions[cf].matches.size()<15) continue;
        //go to the map looking for more matches
        Solutions[cf].matches= TheMap->matchFrameToMapPoints ( TheMap->TheKpGraph.getNeighborsVLevel2( kf,true) , curFrame,  Solutions[cf].pose ,_params.maxDescDistance*2, 2.5,true);
        if (Solutions[cf].matches.size()<30) continue;

        //now refine
        PnPSolver::solvePnp(curFrame,TheMap,Solutions[cf].matches,Solutions[cf].pose);
         if (Solutions[cf].matches.size()<30) continue;
        Solutions[cf]. ids=curFrame.ids;
        for(auto match: Solutions[cf].matches)
            Solutions[cf].ids[ match.queryIdx]=match.trainIdx;
    }

    //take the solution with more matches
    std::remove_if(Solutions.begin(),Solutions.end(),[](const kp_reloc_solution &a){return a.matches.size()<=30;});
    std::sort( Solutions.begin(),Solutions.end(),[](const kp_reloc_solution &a,const kp_reloc_solution &b){return a.matches.size()>b.matches.size();});

   return Solutions;


}

bool System::relocalize_withkeypoints( Frame &curFrame,se3 &pose_f2g_out , const std::set<uint32_t> &excluded ){

    auto Solutions=relocalization_withkeypoints_(curFrame,pose_f2g_out,excluded);
    if (Solutions.size()==0) return false;
    if (Solutions[0].matches.size()>30){
        pose_f2g_out=Solutions[0].pose;
        curFrame.ids=Solutions[0].ids;
        return true;
    }
    else return false;

}

bool System::relocalize_withmarkers( Frame &f,se3 &pose_f2g_out ){
     if (f.markers.size()==0)return false;
    //find the visible makers that have valid 3d location
    vector< uint32_t> valid_markers_found;
    for(  auto &m:f.markers){//for ech visible marker
        auto map_marker_it=TheMap->map_markers.find(m.id);
        if(map_marker_it!=TheMap->map_markers.end())//if it is in the map
            if ( map_marker_it->second.pose_g2m.isValid() )//and its position is valid
                valid_markers_found.push_back( m.id);
    }
    if (valid_markers_found.size()==0)return false;

  pose_f2g_out= TheMap->getBestPoseFromValidMarkers(f,valid_markers_found,_params.aruco_minerrratio_valid);
  return pose_f2g_out.isValid();

}



 bool System::relocalize(Frame &f, se3 &pose_f2g_out){
    pose_f2g_out=se3();
    if( _params.reLocalizationWithMarkers){
        if ( relocalize_withmarkers(f,pose_f2g_out)) return true;
    }
    if( _params.reLocalizationWithKeyPoints){
        if (relocalize_withkeypoints(f,pose_f2g_out)) return true;
    }
    return false ;
}


//given lastframe, current and the map of points, search for the map points  seen in lastframe in current frame
//Returns query-(id position in the currentFrame). Train (id point in the TheMap->map_points)
std::vector<cv::DMatch> System::matchMapPtsOnPrevFrame(Frame & curframe, Frame &prev_frame, float minDescDist, float maxReprjDist)
{

    //compute the projection matrix
    std::vector<cv::DMatch> matches;

    for(size_t i=0;i<prev_frame.ids.size();i++){
        uint32_t pid=prev_frame.ids[i];
        if (pid!=std::numeric_limits<uint32_t>::max()){//the point was found in prev frame
            if( TheMap->map_points.is(pid)){
                MapPoint &mPoint=TheMap->map_points[pid];
                if (mPoint.isBad()) continue;
                auto p2d=curframe.project(mPoint.getCoordinates(),true,true);
                if ( isnan(p2d.x))continue;

                //scale factor by octave
                float sc=curframe.scaleFactors[ prev_frame.und_kpts[i].octave];
                int octave=prev_frame.und_kpts[i].octave;
                std::vector<uint32_t> kpts_neigh=curframe.getKeyPointsInRegion(p2d,maxReprjDist*sc,octave,octave);

                float bestDist=minDescDist+0.01,bestDist2=std::numeric_limits<float>::max();
                uint32_t bestIdx=std::numeric_limits<uint32_t>::max();
                //only consider these within neighbor octaves and with a minimun theshold distance
                for(auto kp:kpts_neigh){
                    if (  curframe.und_kpts[ kp].octave == prev_frame.und_kpts[i].octave ){
                        float descDist= MapPoint::getDescDistance(prev_frame.desc,i, curframe.desc,kp);
                        if ( descDist<bestDist){
                            bestDist=descDist;
                            bestIdx=kp;
                        }
                        else if (descDist<bestDist2){
                            bestDist2=descDist;
                        }
                    }
                }
                //now, only do the match if the ratio between first and second is good
                if (bestIdx!=std::numeric_limits<uint32_t>::max() && bestDist< 0.7*bestDist2){
                    cv::DMatch mt;
                    mt.queryIdx = bestIdx; // kp
                    mt.trainIdx =  mPoint.id;        // map
                    mt.distance = bestDist;
                    matches.push_back( mt );
                }
            }
        }
    }



    _debug_msg("final number of matches= "<<matches.size(),10);
    filter_ambiguous_query(matches);
    _debug_msg("final number of matches2= "<<matches.size(),10);

    return matches;

}

//preconditions
// lastKnownPose and _curKFRef are valid and
se3 System::track(Frame &curframe,se3 lastKnownPose) {




    __TSLAM_ADDTIMER__
        //first estimate current pose
    std::vector<cv::DMatch> map_matches;

    se3 estimatedPose=lastKnownPose;    //estimate the current pose using motion model


    //search for the points matched and obtain an initial  estimation if there are map points
     if (TheMap->map_points.size()>0){
         if (!velocity.empty())
             estimatedPose=velocity*estimatedPose.convert();
        curframe.pose_f2g=estimatedPose;

        //find mappoints found in previous frame
        map_matches=matchMapPtsOnPrevFrame(curframe,_prevFrame,_params.maxDescDistance*1.5,_params.projDistThr);
        __TSLAM_TIMER_EVENT__("matchMapPtsOnPrevFrame");
        int nvalidMatches=0;
        if (map_matches.size()>30  ){//do pose estimation and see if good enough
            auto estimatedPose_aux=estimatedPose;
            nvalidMatches=PnPSolver::solvePnp( curframe,TheMap,map_matches,estimatedPose_aux,_curKFRef);
            if (nvalidMatches>30) estimatedPose=estimatedPose_aux;
        }
//        else{//if not enough matches found from previous, do a search to reference keyframe
//            //match this and reference keyframe
//            FrameMatcher fmatcher(FrameMatcher::TYPE_FLANN);//use flann since BoW is not yet computed and can not be used
//            fmatcher.setParams( TheMap->keyframes[_curKFRef],FrameMatcher::MODE_ASSIGNED,_params.maxDescDistance*2,0.6,true,3);//search only amongts mappoints of ref keyframe
//            map_matches=fmatcher.match(curframe,FrameMatcher::MODE_ALL);
//            if(map_matches.size()>30){
//                //change trainIdx from Frame indices to map Ids
//                for(auto &m:map_matches)
//                    m.trainIdx= TheMap->keyframes[_curKFRef].ids[m.trainIdx];
//                auto estimatedPose_aux=estimatedPose;
//                nvalidMatches=PnPSolver::solvePnp( curframe,TheMap,map_matches,estimatedPose_aux,_curKFRef);
//                if (nvalidMatches>30) estimatedPose=estimatedPose_aux;
//            }
//            else nvalidMatches=0;
//        }



        __TSLAM_TIMER_EVENT__("poseEstimation");
        _debug_msg("matchMapPtsOnFrames = "<<map_matches.size(),10);
        float projDistance;

        if ( nvalidMatches>30) {//a good enough initial estimation is obtained
            projDistance=4;//for next refinement
            for(auto m:  map_matches ){//mark these as already being used and do not consider in next step
                TheMap->map_points[m.trainIdx].lastFIdxSeen= curframe.fseq_idx;
                TheMap->map_points[m.trainIdx].setVisible();
            }
        }
        else{
            map_matches.clear();
            projDistance=_params.projDistThr;//not good previous step, do a broad search in the map
        }

        //get the neighbors and current, and match more points in the local map
        auto map_matches2 = TheMap->matchFrameToMapPoints ( TheMap->TheKpGraph.getNeighborsVLevel2( _curKFRef,true) , curframe,  estimatedPose ,_params.maxDescDistance*2, projDistance,true);
        //add this info to the map and current frame
        _debug_msg("matchMapPtsOnFrames 2= "<<map_matches2.size(),10);
        map_matches.insert(map_matches.end(),map_matches2.begin(),map_matches2.end());
        _debug_msg("matchMapPtsOnFrames final= "<<map_matches.size(),10);
        __TSLAM_TIMER_EVENT__("matchMapPtsOnFrames");
        filter_ambiguous_query(map_matches);
    }

     int nValidMatches=PnPSolver::solvePnp( curframe,TheMap,map_matches,estimatedPose,_curKFRef);
    __TSLAM_TIMER_EVENT__("poseEstimation");

    //determine if the pose estimated is reliable
    //is aruco accurate
    bool isArucoTrackOk=false;
    if ( curframe.markers.size()>0){
        int nvalidMarkers=0;
        //count how many reliable marker are there
        for(size_t  i=0;i<  curframe.markers.size();i++){
            //is the marker in the map with valid pose
            auto mit=TheMap->map_markers.find( curframe.markers[i].id);
            if(mit==TheMap->map_markers.end())continue;//is in map?
            if (!mit->second.pose_g2m.isValid())continue;//has a valid pose?
            nvalidMarkers++;
            //is it observed with enough reliability?
            if (curframe.markers[i].poses.err_ratio < _params.aruco_minerrratio_valid) continue;
            isArucoTrackOk=true;
            break;
        }
        if (nvalidMarkers>1) isArucoTrackOk=true;//lets consider that we only need 2 valid markers for good result

    }


    _debug_msg_("A total of "<<nValidMatches <<" good matches found and isArucoTrackOk="<<isArucoTrackOk);
    if (nValidMatches<30 && !isArucoTrackOk){
        _debug_msg_("need relocatization");
     return se3();
    }


    /// update mappoints knowing now the outliers. It olny affects to map points
    for(size_t i=0;i<map_matches.size();i++){
            TheMap->map_points[map_matches[i].trainIdx].setSeen();
            curframe.ids[ map_matches[i].queryIdx]=map_matches[i].trainIdx;
            if (map_matches[i].imgIdx==-1)//is a bad match(outlier)
                curframe.flags[ map_matches[i].queryIdx].set(Frame::FLAG_OUTLIER,true);
    }




    return estimatedPose;
}




void System::drawMatchesAndMarkersInInputImage( cv::Mat &image,float inv_ScaleFactor)const{


    int size=float(image.cols)/640.f;
    cv::Point2f psize(size,size);
    bool drawAllKp=false;

     if (currentState==STATE_TRACKING){
        for(size_t i=0;i<_cFrame.ids.size();i++)
            if (_cFrame.ids[i]!=std::numeric_limits<uint32_t>::max()){
                if (!TheMap->map_points.is( _cFrame.ids[i])) continue;
                cv::Scalar color(0,255,0);
                if (!TheMap->map_points[_cFrame.ids[i]].isStable()) {color=cv::Scalar(0,0,255);}
                cv::rectangle(image,inv_ScaleFactor*(_cFrame.kpts[i]-psize),inv_ScaleFactor*(_cFrame.kpts[i]+psize),color,size);
            }
            else if( drawAllKp){
                cv::Scalar color(255,0,0);
                cv::rectangle(image,inv_ScaleFactor*(_cFrame.kpts[i]-psize),inv_ScaleFactor*(_cFrame.kpts[i]+psize),color,size);

            }
    }
    else if(TheMap->isEmpty()){
        for( auto p: _cFrame.kpts)
            cv::rectangle(image,inv_ScaleFactor*(p-psize),inv_ScaleFactor*(p+psize),cv::Scalar(255,0,0),size);
    }

    //now, draw markers found
    for(auto aruco_marker:_cFrame.markers){
         //apply distortion back to the points
        cv::Scalar color=cv::Scalar(0,244,0);
        if( TheMap->map_markers.count(aruco_marker.id)!=0){
            if( TheMap->map_markers.at(aruco_marker.id).pose_g2m.isValid())
                color=cv::Scalar(255,0,0);
            else
                color=cv::Scalar(0,0,255);
        }
        //scale
        for(auto &p:aruco_marker.corners) p*=inv_ScaleFactor;
        for(auto &p:aruco_marker.und_corners) p*=inv_ScaleFactor;
        //draw
        aruco_marker.draw(image,color);
    }

}



void System::globalOptimization(){
    _debug_exec( 10, saveToFile("preopt.ucs"););

    auto opt=GlobalOptimizer::create(_params.global_optimizer);
    GlobalOptimizer::ParamSet params( debug::Debug::getLevel()>=11 );
    params.fixFirstFrame=true;
    params.nIters=10;
    params.markersOptWeight= getParams().markersOptWeight;
    params.minMarkersForMaxWeight=getParams().minMarkersForMaxWeight;
    params.InPlaneMarkers=getParams().inPlaneMarkers;

    _debug_msg_("start initial optimization="<<TheMap->globalReprojChi2());

    opt->optimize(TheMap,params );
      _debug_msg_("final optimization="<<TheMap->globalReprojChi2() <<" npoints="<< TheMap->map_points.size());


    TheMap->removeBadAssociations(opt->getBadAssociations(),2);
    _debug_msg_("final points ="<< TheMap->map_points.size());



    _debug_exec( 10, saveToFile("postopt.ucs"););

}


uint32_t System::getLastProcessedFrame()const{return _cFrame.fseq_idx;}


void System::setMode(MODES mode){
    currentMode=mode;
}





 void System::clear(){
     TheMapManager=std::make_shared<MapManager>();
     isInitialized=false;
     currentState=STATE_LOST;
     TheMap.reset();
     map_initializer=std::make_shared<MapInitializer>();
     velocity=cv::Mat();
     lastKFReloc=-1;
 }


 void System::saveToFile(string filepath){


     waitForFinished();

     //open as input/output
     fstream file(filepath,ios_base::binary|ios_base::out );
     if(!file)throw std::runtime_error(string(__PRETTY_FUNCTION__)+"could not open file for writing:"+filepath);

     //write signature
     io_write<uint64_t>(182313,file);

     TheMap->toStream(file);

      //set another breaking point here
     _params.toStream(file);
     file.write((char*)&_curPose_f2g,sizeof(_curPose_f2g));
     file.write((char*)&_curKFRef,sizeof(_curKFRef));
     file.write((char*)&isInitialized,sizeof(isInitialized));
     file.write((char*)&currentState,sizeof(currentState));
     file.write((char*)&currentMode,sizeof(currentMode));
     _cFrame.toStream(file);
     _prevFrame.toStream(file);
     fextractor->toStream(file);
     TheMapManager->toStream(file);
     marker_detector->toStream(file);
     toStream__(velocity,file);
     file.write((char*)&lastKFReloc,sizeof(lastKFReloc));

     file.write((char*)&totalNFramesProcessed,sizeof(totalNFramesProcessed));

     file.flush();
 }

 void System::readFromFile(string filepath){
     ifstream file(filepath,ios::binary);
     if(!file)throw std::runtime_error(string(__PRETTY_FUNCTION__)+"could not open file for reading:"+filepath);

     if ( io_read<uint64_t>(file)!=182313)  throw std::runtime_error(string(__PRETTY_FUNCTION__)+"invalid file type:"+filepath);

     TheMap=std::make_shared<Map>();
     TheMap->fromStream(file);

     _params.fromStream(file);
      file.read((char*)&_curPose_f2g,sizeof(_curPose_f2g));
     file.read((char*)&_curKFRef,sizeof(_curKFRef));
     file.read((char*)&isInitialized,sizeof(isInitialized));
     file.read((char*)&currentState,sizeof(currentState));
     file.read((char*)&currentMode,sizeof(currentMode));

     _cFrame.fromStream(file);
     _prevFrame.fromStream(file);

     fextractor->fromStream(file);
     TheMapManager->fromStream(file);
     marker_detector->fromStream(file);
     fromStream__(velocity,file);
     file.read((char*)&lastKFReloc,sizeof(lastKFReloc));
     file.read((char*)&totalNFramesProcessed,sizeof(totalNFramesProcessed));

 //    setParams(TheImageParams,_params);


 }


 } //namespace
