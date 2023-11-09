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
#include "loopdetector.h"
#include "optimization/graphoptsim3.h"
#include "basictypes/misc.h"
#include "system.h"
#include "basictypes/timers.h"
#include "optimization/pnpsolver.h"
#include "basictypes/io_utils.h"
#include "basictypes/hash.h"
#include "utils/framematcher.h"
#include "basictypes/osadapter.h"
#include <xflann/xflann.h>
namespace  tslam {



void LoopDetector::setParams(std::shared_ptr<Map> map){
    TheMap=map;
}

LoopDetector::LoopClosureInfo  LoopDetector::detectLoopFromMarkers(Frame &frame, int32_t curRefKf){
    assert(TheMap);
    __TSLAM_ADDTIMER__


    vector<LoopClosureInfo> loopClosurePossibilities;
    //first, try with markers
    if (TheMap->map_markers.size()>0 && frame.markers.size()!=0){
        loopClosurePossibilities= detectLoopClosure_Markers(frame,curRefKf);
        __TSLAM_TIMER_EVENT__("detectLoopClosure_Markers");
    }

    if (loopClosurePossibilities.size()==0) return  LoopDetector::LoopClosureInfo();

    for(auto &lc:loopClosurePossibilities) solve(frame,lc);

    int best_solution=0;
    if (loopClosurePossibilities.size()>1){
        double err1=testCorrection(loopClosurePossibilities[0]);
        double err2=testCorrection(loopClosurePossibilities[1]);
        if (err2<err1) best_solution=1;

    }
    __TSLAM_TIMER_EVENT__("test solutions");

    //now, lets find keypoint matches between the sides of the loop


    auto &LoopSolution=loopClosurePossibilities[best_solution];
    LoopSolution.map_matches = TheMap->matchFrameToMapPoints(TheMap->TheKpGraph.getNeighborsV(LoopSolution.matchingFrameIdx,true),
                                                               frame,  LoopSolution.expectedPos ,System::getParams().maxDescDistance*1.5, 2.5,false,true);

    return LoopSolution;
}



LoopDetector::LoopClosureInfo LoopDetector::detectLoopFromKeyPoints(Frame &frame, int32_t curRefKf ){
        __TSLAM_ADDTIMER__
        vector<LoopClosureInfo> loopClosurePossibilities;
    if ( frame.ids.size()!=0){
        loopClosurePossibilities=detectLoopClosure_KeyPoints(frame,curRefKf);
        __TSLAM_TIMER_EVENT__("detectLoopClosure_KeyPoints");
    }
    if (loopClosurePossibilities.size()==0) return LoopDetector::LoopClosureInfo() ;
    else{
        _debug_msg_("Loop closure found with frame "<<TheMap->keyframes[ loopClosurePossibilities[0].matchingFrameIdx].fseq_idx);
        solve(frame,loopClosurePossibilities[0]);
         return loopClosurePossibilities[0];
    }
}






vector<LoopDetector::LoopClosureInfo>  LoopDetector::detectLoopClosure_Markers(Frame & frame,int64_t curkeyframe){

    auto WithValidPose=[&](const vector<uint32_t> &v){
        int s=0;
        for(auto id:v)
            if( TheMap->map_markers[id].pose_g2m.isValid())  s++;
        return s;
    };
    //we detect a loop closure if there a valid marker in the view, with good localization, that is not connected in the graph to curkeyframe
    //so first, see if the case holds
    LoopClosureInfo lci;
    std::set<uint32_t> l1neigh=TheMap->getNeighborKeyFrames(curkeyframe,true);
     //find all markers in the neighbors and in this
    std::set<uint32_t> visibleMarkers;
    for(auto n:l1neigh)
        for(auto m:TheMap->keyframes[n].markers)
            visibleMarkers.insert(m.id);


    //now, find these causing the loop closure
    vector<uint32_t> markersCausingLoopClosure;
    for(size_t mi=0;mi< frame.markers.size();mi++){
        auto &marker=frame.markers[mi];
         if (  visibleMarkers.count(marker.id)!=0)continue;//has already been seen
        auto mit=TheMap->map_markers.find(marker.id);//is in the system???
         if(mit!=TheMap->map_markers.end()){//yes
           //  if (mit->second.pose_g2m.isValid()){//is valid??
                 //                        if ( frame.markers_solutions[mi].err_ratio>_params.aruco_minerrratio_valid) //is it reliable?
                markersCausingLoopClosure.push_back(marker.id);
            }
    }


    if ( WithValidPose(markersCausingLoopClosure)==0) return  {};


    //now, well try to solve the problem we have.
    //current frame has two possible locations,
    //first, the one estimated with drift (which is in frame)
    //and the one calculated using the markers seen already (expected)
     //determine this original location using the frames that caused the loop closure
    vector<se3> expectedposes;

    se3 safe_expectedpose=TheMap->getBestPoseFromValidMarkers(frame,markersCausingLoopClosure,4);
    //a minimum confidence is required in order to do the loop closure.
    if (   !safe_expectedpose.isValid()  && WithValidPose(markersCausingLoopClosure)<3){
     //the markers are there, but the confidence is low. Better not to do it. Lets remove the markers from the frame
        //to avoid ruining the process
        _debug_msg_("Unsafe Loop closure. Removing Markers involved ");
        for(auto m:markersCausingLoopClosure){
            auto it=std::find_if( frame.markers.begin(),frame.markers.end(), [&](const tslam::MarkerObservation &mo){return mo.id==m;});
            if( it!=frame.markers.end())
                frame.markers.erase(it);
        }
        return {};
    }



     //is the pose  good enough?
    if (   safe_expectedpose.isValid())
        expectedposes.push_back(safe_expectedpose);
    //if not, it is safer to fo optimization using the two possible ways
    else{
        int bestM=0;
        for(size_t i=1;i< markersCausingLoopClosure.size();i++)
            if ( frame.getMarkerPoseIPPE(markersCausingLoopClosure[i]).err_ratio>frame.getMarkerPoseIPPE(markersCausingLoopClosure[bestM]).err_ratio )
                bestM=i;
        //compute current pose based on the two solutions
        //F2G
        int bestMId=markersCausingLoopClosure[bestM];
        cv::Mat  invM=TheMap->map_markers[bestMId].pose_g2m.inv() ;
        se3 pose1=       se3(frame.getMarkerPoseIPPE(bestMId).sols[0]* invM);
        se3 pose2=       se3(frame.getMarkerPoseIPPE(bestMId).sols[1]* invM);
        expectedposes.push_back(pose1);
        expectedposes.push_back(pose2);

    }
 //    if(!expectedpose.isValid());



     //so, the frame is at frame.pose_f2g but it should be at orgpose also
     //loop closure consists in making these two estimations identical by distributing the errors equally amongst the nodes in the
     //path connecting the orignal location and current one
     //But, which is the 'original location'? There are many possibilities in our problem, since the marker has been seen from
     //many other frames probably.
     //So, let us take as origin node the first one in  the video in which it was seen
     uint32_t minFrameSeqIdx=std::numeric_limits<uint32_t>::max(),minFrameIdx=std::numeric_limits<uint32_t>::max();
     for(auto m:markersCausingLoopClosure){
         assert(TheMap->map_markers.count(m)!=0);//the marker must be and be valid
         assert(TheMap->map_markers.at(m).pose_g2m.isValid());
         for(auto frame_idx:TheMap->map_markers[m].frames){
             if ( minFrameSeqIdx>TheMap->keyframes[frame_idx].fseq_idx){
                 minFrameSeqIdx=TheMap->keyframes[frame_idx].fseq_idx;
                 minFrameIdx=frame_idx;
             }
         }
     }

vector<LoopClosureInfo> vlci;
for(size_t i=0;i<expectedposes.size();i++){
    LoopClosureInfo lci;
    lci.curRefFrame=curkeyframe;
    lci.expectedPos=expectedposes[i];
    lci.matchingFrameIdx=minFrameIdx;
    vlci.push_back(lci);
}

    return vlci;

}

void LoopDetector::solve(Frame &frame,   LoopClosureInfo &loopc_info){
    CovisGraph &TheGraph=TheMap->TheKpGraph;


     if ( frame.idx==std::numeric_limits<uint32_t>::max())//in case it is from markers, get an id(required for optimization with g2o)
        frame.idx=TheMap->getNextFrameIndex();

    vector<pair<uint32_t,uint32_t> > edges=TheGraph.getAllEdges();
    if (!TheGraph.isEdge(loopc_info.curRefFrame,frame.idx));
    edges.push_back({loopc_info.curRefFrame,frame.idx});
    auto &optimPoses=loopc_info.optimPoses;
    std::map<uint64_t,float> edge_weights;
    float maxWeight=0;
    for(auto e:edges){
        if ( optimPoses.count(e.first)==0 && TheMap->keyframes.is(e.first))
            optimPoses[e.first]=TheMap->keyframes[e.first].pose_f2g;
        if ( optimPoses.count(e.second)==0 && TheMap->keyframes.is(e.second))
            optimPoses[e.second]=TheMap->keyframes[e.second].pose_f2g;
        float eWeight=0;
        if ( TheMap->TheKpGraph.isEdge(e.first,e.second))//case when loop closure detection is given by marker (excluded)
            eWeight=TheMap->TheKpGraph.getEdge(e.first,e.second);
        maxWeight=std::max(eWeight,maxWeight);
        edge_weights[ CovisGraph::join(e.first,e.second)]=eWeight;
    }

    if (!TheGraph.isEdge(frame.idx,loopc_info.matchingFrameIdx)){
        edges.push_back({frame.idx,loopc_info.matchingFrameIdx});
        edge_weights[ CovisGraph::join(frame.idx,loopc_info.matchingFrameIdx)]=maxWeight;
    }
    optimPoses[frame.idx]=frame.pose_f2g;


#pragma message "warning: must check fixscale"

    loopClosurePathOptimizationg2o(edges,frame.idx,loopc_info.matchingFrameIdx,loopc_info.expectedPos, optimPoses,false);


}

void LoopDetector::correctMap(  const LoopClosureInfo &lcsol){
    //move markers
    auto Sim3ToSE3=[](const cv::Mat &Sim3){
        float sc= cv::norm( Sim3.col(0));
        cv::Mat RT=Sim3;
        cv::Mat R=Sim3.rowRange(0,3).colRange(0,3);
        R=(1./sc)*R;
        return RT;
    };


    //move markers
    for(auto &id_marker:TheMap->map_markers){
        Marker &marker=id_marker.second;
        assert(marker.frames.size()>0);
        if (!marker.pose_g2m.isValid()) continue;
       //take an observing frame and transform the marker

        bool isBeingCorrected=false;
        for(auto fidx:marker.frames){
            if( lcsol.optimPoses.count(fidx)==0)continue;
            const Frame &frame=TheMap->keyframes[fidx];
            //move the marker pose to the frame
            cv::Mat f2m=frame.pose_f2g*  marker.pose_g2m;//marker to frame transform
            //remove the scale from the optim pose
            auto RT_f2g=Sim3ToSE3(lcsol.optimPoses.at(fidx));//new optim pose for the fraem
            //now, obtain the new marker pose
            marker.pose_g2m=      RT_f2g.inv() *  f2m;//new marker pose
            isBeingCorrected=true;

            break;
        }
        if (!isBeingCorrected)throw std::runtime_error(string(__PRETTY_FUNCTION__)+"Internal error. Could not correct marker");
        assert(isBeingCorrected);
    }

     //move points
    for(auto &mp:TheMap->map_points){
            //move to one of the observing frames and correct using its pose
            Frame& kf=TheMap->keyframes[ mp.getObservingFrames().front().first];
            cv::Point3f p3d=kf.pose_f2g*mp.getCoordinates();
            //now, correct with new sim3 transform
            cv::Mat Sim3=lcsol.optimPoses.at(kf.idx);
            float sc= cv::norm( Sim3.col(0));
            cv::Mat R=Sim3.rowRange(0,3).colRange(0,3);
            R*=(1./sc);
            cv::Mat RTinv=Sim3.inv();
            R=RTinv.rowRange(0,3).colRange(0,3);
            R*=(1./sc);
            mp.setCoordinates( RTinv*p3d );
    }

    //move keyframes
    for(const auto op:lcsol.optimPoses){
        if ( TheMap->keyframes.count(op.first)){
            //get normalized version of transform
            TheMap->keyframes[op.first].pose_f2g=Sim3ToSE3(op.second  );// RT
        }
    }
}


double LoopDetector::testCorrection(const LoopClosureInfo &lcsol){

    std::map<uint32_t,se3> marker_poses;
    std::map<uint32_t,cv::Point3f> point_coordinates;
    std::map<uint32_t,Se3Transform> frame_poses;

    for(auto m:TheMap->map_markers)
        marker_poses[m.first]=m.second.pose_g2m;
    for(auto p:TheMap->map_points)
        point_coordinates.insert({p.id,p.getCoordinates()});
    for(auto f:TheMap->keyframes)
        frame_poses[f.idx]=f.pose_f2g;

    correctMap(lcsol);


    //compute the error
    vector<uint32_t> keyframes;
    for(auto kf:lcsol.optimPoses )
        if (TheMap->keyframes.is(kf.first))
            keyframes.push_back(kf.first);
    auto err=TheMap->globalReprojChi2(keyframes,0,0,true,true);
    _debug_msg("err sol ="<<err ,10);

    //restore
    for(auto &m:TheMap->map_markers)
        m.second.pose_g2m = marker_poses[m.first];
    for(auto &k:TheMap->keyframes)
        k.pose_f2g= frame_poses[k.idx];
    for(auto &mp:TheMap->map_points)
        mp.setCoordinates(point_coordinates[mp.id]);

    return err;
}
vector<cv::DMatch> LoopDetector::matchFrameToKeyFrame(std::shared_ptr<Map> map, const Frame&, uint32_t kf, float minDescDist,void*xfindexPtr)
{
    xflann::Index *xfindex=(  xflann::Index *) xfindexPtr;
    //take all the descriptors of the mappoints seen and put them in a matrix. Then create a index for fast search
    Frame &kframe=map->keyframes[kf];
    //save the indices of the map points in mpts
    vector<uint32_t> mpts;mpts.reserve(kframe.und_kpts.size());
    for(auto kpidx:kframe.ids){
        if (kpidx!=std::numeric_limits<uint32_t>::max())
            mpts.push_back(kpidx);
    }
    if (mpts.size()==0)return{};
    //create the matrix MapPointDescriptors where the descritors are copied
    cv::Mat ADescriptor;
    map->map_points[ mpts[0]].getDescriptor(ADescriptor);//take a descriptor to know its size and type
    cv::Mat MapPointDescriptors(mpts.size(),ADescriptor.cols,ADescriptor.type() );
    //start the copy
    for(size_t i=0;i<mpts.size();i++){
        cv::Mat row=MapPointDescriptors.row(i);
        map->map_points[ mpts[i]].getDescriptor(row);
    }

    //now, do a search amonts the descriptors in this the search frame f

    cv::Mat indices,distances;
    xfindex->search(MapPointDescriptors,2,indices,distances,xflann::KnnSearchParams(32,true));
    if ( distances.type()==CV_32S) distances.convertTo(distances,CV_32F);

    float distThres= minDescDist;
    float nn_match_ratio=0.9;

    vector<cv::DMatch> matches;
    for(int i = 0; i < indices.rows; i++) {
        if ( distances.at<float>(i,0)<distThres){
            if(distances.at<float>(i,0)  < float(nn_match_ratio * float(distances.at<float>(i,1)))) {
                cv::DMatch  match;
                match.queryIdx= indices.at<int>(i,0);
                match.trainIdx= mpts[ i];
                match.distance=distances.at<float>(i,0);
                matches.push_back(match);
            }
        }
    }
    filter_ambiguous_query(matches);

    return matches;
}

std::vector<LoopDetector::LoopClosureInfo> LoopDetector::detectLoopClosure_KeyPoints( Frame &frame, int32_t curRefKf){
    auto _to_string=[](const uint32_t&val){ std::stringstream sstr;sstr<<val;return sstr.str();};

    __TSLAM_ADDTIMER__
            if (!System::getParams().detectKeyPoints) return {};
    if (TheMap->TheKFDataBase.isEmpty())return {};

    auto Neighbors=TheMap->TheKpGraph.getNeighbors(curRefKf,true);
    if (Neighbors.size()==TheMap->keyframes.size()) return {};//case in which all keyframes are connected
    __TSLAM_TIMER_EVENT__("find neighbors");
    // cout<<"neighbors=";for(auto n:Neighbors)cout<<n<<" ";cout<<endl;
    float minScore = 1;
    for(auto neigh:Neighbors)
        minScore=std::min(minScore, TheMap->TheKFDataBase.score( frame,TheMap->keyframes[neigh]));
    __TSLAM_TIMER_EVENT__("min score in "+_to_string(Neighbors.size())+" neighbors");
    auto candidates=TheMap->TheKFDataBase.relocalizationCandidates(frame,TheMap->keyframes,TheMap->TheKpGraph,true,minScore/2.,Neighbors);
 //   _debug_exec_(cout<<"candidates:";for(auto c:candidates)cout<<c<<" ";cout<<endl;);
    if(candidates.size()==0) return {};
    //SCORE TO CANDIDATES
    __TSLAM_TIMER_EVENT__("find best candidates");



    //check for any candidate has strong evidence
    struct Candidate{
        Candidate(uint32_t f){frame=f;}
        uint32_t frame;cv::Mat pose=cv::Mat();uint32_t nInliers=0;bool remove=false;
        vector<cv::DMatch> map_matches;
    };
    vector<Candidate> goodLoopCandidates;
    for(auto lc:candidates)
            goodLoopCandidates.push_back(Candidate(lc));





    FrameMatcher FMatcher;//(FrameMatcher::TYPE_FLANN);
    FMatcher.setParams(frame,FrameMatcher::MODE_ALL,System::getParams().maxDescDistance*2);
#pragma message "warning: Check the loop detector is correct with the FrameMatcher"
  //  xflann::Index FrameXFlannIndex (frame.desc,xflann::HKMeansParams(8,0));
    __TSLAM_TIMER_EVENT__("build index  ");

    for(auto &cand:goodLoopCandidates){

        vector<cv::Point2f> points2d;
        vector<cv::Point3f> points3d;
        vector<cv::DMatch> matches;
        if(1){//new method using frame matcher (needs testing)
            auto &KFrame=TheMap->keyframes[cand.frame];
            matches=FMatcher.match(KFrame,FrameMatcher::MODE_ASSIGNED);
            //change trainIdx and queryIdx to match the  solvePnpRansac requeriments
            _debug_msg_("cand="<<cand.frame<<" matches="<<matches.size());
            __TSLAM_TIMER_EVENT__(" matchFrameToKeyFrame");
            if(matches.size()<30){ //if not enough matches, mark for removal
                cand.remove=true;
                continue;
            }
            for(auto m:matches){
                points2d.push_back(frame.und_kpts[m.trainIdx].pt);
                points3d.push_back(TheMap->map_points[ KFrame.ids[ m.queryIdx] ].getCoordinates());
            }
        }
//        else{

//            matches=matchFrameToKeyFrame(TheMap,frame,cand.frame,System::getParams().maxDescDistance*2,&FrameXFlannIndex);
//            _debug_msg_("cand="<<cand.frame<<" matches="<<matches.size());
//            __TSLAM_TIMER_EVENT__(" matchFrameToKeyFrame");
//            if(matches.size()<30){ //if not enough matches, mark for removal
//                cand.remove=true;
//                continue;
//            }

//            for(auto m:matches){
//                points2d.push_back(frame.und_kpts[m.queryIdx].pt);
//                points3d.push_back(TheMap->map_points[ m.trainIdx].getCoordinates());
//            }
//        }


        cv::Mat rv,tv;
        vector<int> inliers;
        //NEEDS TO REWRITE CON CONSIDER SCALE IN PROJECTION ERROR
        cout<<"PNPR="<<points3d.size()<<endl;
        cv::solvePnPRansac(points3d,points2d, frame.imageParams.CameraMatrix,cv::Mat::zeros(1,5,CV_32F),rv,tv,false,100,2.5,0.99,inliers,cv::USAC_MAGSAC);
        _debug_msg_("  inliers="<<inliers.size());
        __TSLAM_TIMER_EVENT__(" pnpransac");
        if (inliers.size()<15 ){
            cand.remove=true;
            continue;
        }
        cand.nInliers=inliers.size();
        cand.pose=getRTMatrix(rv,tv,CV_32F);
        //remove the outliers
        cand.map_matches.reserve(inliers.size());
        for(auto in:inliers)
            cand.map_matches.push_back( matches[in] );
    }


    goodLoopCandidates.erase(std::remove_if(goodLoopCandidates.begin(),goodLoopCandidates.end(),[](const Candidate& c){return c.remove;}),goodLoopCandidates.end());

    if (goodLoopCandidates.size()==0) return  {};
    //sort by number of inliers
    std::sort(goodLoopCandidates.begin(),goodLoopCandidates.end(),[](const Candidate &c1,const Candidate &c2){return c1.nInliers>c2.nInliers;});





    //Find more points among the neighbors
    for(auto &good_cand:goodLoopCandidates){
        //refine even further the pose
        for(auto &mp:TheMap->map_points) mp.lastFIdxSeen=std::numeric_limits<uint32_t>::max();
        //set as used the already seen
        good_cand.map_matches =TheMap->matchFrameToMapPoints(TheMap->TheKpGraph.getNeighborsV(goodLoopCandidates[0].frame,true), frame,  goodLoopCandidates[0].pose ,System::getParams().maxDescDistance*1.5, 2.5,false,true);
        if(good_cand.map_matches.size()<40) continue;
        se3 pose_io=good_cand.pose;
        PnPSolver::solvePnp(frame, TheMap,good_cand.map_matches, pose_io,curRefKf);
        good_cand.pose=pose_io;
    }
    //sort by number of matches
    std::sort(goodLoopCandidates.begin(),goodLoopCandidates.end(),[](const Candidate &c1,const Candidate &c2){return c1.map_matches.size()>c2.map_matches.size();});

    //if the best has not at least a minimu number of mathces, then not detected
    if ( goodLoopCandidates[0].map_matches.size()<30)return {};

    //analyze the scale

    //prepare the solution
    LoopClosureInfo lci;
    lci.curRefFrame=curRefKf;
    lci.matchingFrameIdx=goodLoopCandidates[0].frame;
    lci.expectedPos=goodLoopCandidates[0].pose;
    lci.map_matches=goodLoopCandidates[0].map_matches;
    return {lci};

}


void LoopDetector::LoopClosureInfo::toStream(ostream &str)const{
    str.write((char*)&curRefFrame,sizeof(curRefFrame));
    str.write((char*)&matchingFrameIdx,sizeof(matchingFrameIdx));
    toStream__(expectedPos,str);
    toStream__(map_matches,str);
    toStream__kv(optimPoses,str);

 }
void LoopDetector::LoopClosureInfo::fromStream(istream &str){
    str.read((char*)&curRefFrame,sizeof(curRefFrame));
    str.read((char*)&matchingFrameIdx,sizeof(matchingFrameIdx));
    fromStream__(expectedPos,str);
    fromStream__(map_matches,str);
    fromStream__kv(optimPoses,str);

}


uint64_t LoopDetector::LoopClosureInfo::getSignature(){
    Hash sig;
    sig+=curRefFrame;
    sig+=matchingFrameIdx;
    sig+=expectedPos;
    for(const auto &e:map_matches){
        sig+=e.distance;
        sig+=e.imgIdx;
        sig+=e.trainIdx;
        sig+=e.queryIdx;
    }
    for(const auto &e:optimPoses) {sig+=e.first;sig+=e.second;}
    return sig;
}

}
