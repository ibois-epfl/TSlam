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
#ifndef _TSLAM_MAP_H
#define _TSLAM_MAP_H
#include <mutex>

#include "map_types/covisgraph.h"
#include "map_types/keyframedatabase.h"
#include "map_types/mappoint.h"
#include "map_types/frame.h"
#include "map_types/marker.h"
#include "basictypes/safemap.h"
#include "tslam_exports.h"
namespace tslam {
/**
 * @brief The Map class represents a map of the environment.
 *
 */
class TSLAM_API Map : public std::enable_shared_from_this<Map>
{
public:
    //set of keypoints of the environments
    ReusableContainer<MapPoint> map_points;
    //set of markers detected in the environment
    SafeMap<uint32_t,Marker> map_markers;
    //Keyframes represent frames acquired by a camera
    FrameSet keyframes;


    //indicates if the map has any keyframe
    inline bool isEmpty()const{return keyframes.size()==0;}
    //clears the map
    void clear();
    //cleans the unused keypoints from the frames. This will reduce a lot of space
    void removeUnUsedKeyPoints();
    //project to another map
    void projectTo(Map &refMap);
    
    /**
     * Saves the (binary) map to a file
     *
     * @param fpath File path
     */
    void saveToFile(string fpath);

    /**
     * Reads the (binary) map from a file
     *
     * @param fpath File path
     */
    void readFromFile(std::string  fpath);

    /**
     * Saves the set of markers to a marker map file that can be used with aruco.
     *
     * @param filepath *.yml (should include the extension)
     */
    void saveToMarkerMap(std::string filepath)const;

    /**
     * Merge mapB into this.
     *
     * @param mapB Another map.
     */
    void merge(std::shared_ptr<Map> mapB);

    /**
     * Run fullba optimization.
     *
     * @param niters Number of iterations(default = 50).
     */
    void optimize(int niters=50);

    /**
     * remove all feature points
     *
     * @param niters Number of iterations(default = 50).
     */
    void removeAllPoints();

    //returns a unique hash value that identify the current status
    uint64_t getSignature(bool print=false)const;
    //exports the  a file for visualization. Possible formats are  .ply and .pcd (pcl library)
    void  exportToFile(std::string filepath,  const cv::Scalar &colorPoints=cv::Scalar(0,0,0),const cv::Scalar &colorKeyFrames=cv::Scalar(255,0,0),const cv::Scalar &colorMarkers=cv::Scalar(0,0,255),const set<uint32_t> &specialKeyFrames={},cv::Scalar colorSpecialKeyFrame=cv::Scalar(-1,-1,-1))const;

    //returns the neighbor frames of a given one using both graphs
    std::set<uint32_t> getNeighborKeyFrames(uint32_t fidx,bool includeFidx);


    //applies the SE3 transform (rotate and translate) to the map.
    void applyTransform(cv::Mat m4x4);
    //scales the map (does not affect to markers!!!)
    //will not be done if there is at least one marker with valid pose and exception is thrown
    void scale(double scale);
    //makes the indicates makerid the center of the map
    bool centerRefSystemInMarker(uint32_t markerId);


   //private use
   inline void lock(const std::string &func_name,const std::string &file, int line  ){IoMutex.lock();}
   inline void unlock(const std::string &func_name,const std::string &file,int line){IoMutex.unlock();}
   //The covisibility graph defines the relationship between keyframes. Two keyframes are connected in the graph if they share mappoints or markers
   CovisGraph TheKpGraph;
   //A database of keyframes employed for relocalization. It uses bag of words technique
   KeyFrameDataBase TheKFDataBase;

    // original priveate
    // Creates a new point in the map and returns a reference to it
    MapPoint & addNewPoint(uint32_t frameSeqId);
    //remove the indicates keyframes and the mappoints with less than minNumProjPoints because of this
    //Makes a full removal!!
    void removeKeyFrames(const std::set<uint32_t> &keyFrames, int minNumProjPoints);

private:

    friend class LoopDetector;
    friend class MapManager;
    friend class System;
    friend class MapInitializer;
    friend class GlobalOptimizerG2O;
    friend class PnPSolver;
    friend class DebugTest;

    //returns the target focus of the map. If -1, it means that there is no keyframe yet
    float getTargetFocus()const;

    void addMapPointObservation(uint32_t mapPoint,uint32_t KeyFrame,uint32_t KfKPtIdx);

    //adds a new frame in the map and returns a reference to it
    Frame &addKeyFrame(const Frame&f );

    //adds a marker too the map and returns a reference to it
    Marker &addMarker(const tslam::MarkerObservation &m);

    //Given a frame with assigned ids, returns the reference keyframe
    int64_t getReferenceKeyFrame(const Frame &frame,   float minDist=std::numeric_limits<float>::max());


    void addMarkerObservation(uint32_t marker,uint32_t KeyFrame);

    //remove the indicated points if present
    void removePoint(uint32_t pid,bool fullRemoval=true);

    //remove the indicated points
    template<typename It>
    void removePoints(It beg, It end,bool fullRemoval=true){    for(auto it=beg;it!=end;it++) removePoint(*it,fullRemoval);}
    
    //remove mappoint observation
    //returns true if the point has also been removed
    bool removeMapPointObservation(uint32_t kpt, uint32_t frame, uint32_t minNumProjPoints);
    //remove point association and returns the number of frames the point remains visible from
    //If minNumProj>0, the point will be removed if the number of observations falls below this value
   // uint32_t removePointObservation(uint32_t point,uint32_t frame,int32_t minNumProj=0 );
    //returns the median depth of the indicated frame
    float  getFrameMedianDepth(uint32_t frame_idx);

    // void removeOldKeyFrames(const std::set<uint32_t> &keyFrames, int mapKFNumber,int keepAmount=20);

    //returns the expected id of the next frame to be inserted
    uint32_t getNextFrameIndex()const;


    //fuses two map points removing the second one.
    void fuseMapPoints(uint32_t mp1, uint32_t mp2 ,bool fullRemovePoint2=false );


    bool hasPointStereoObservations(uint32_t mpIdx)const;

    //checks that data is consistent
    bool checkConsistency(bool checkCovisGraph=false, bool useMutex=true);


    //remove connections of  neighbors of the kf with a weight smaller than specified
    void removeWeakConnections(uint32_t kf,float minWeight);
    //---------------------
    //serialization routines
    void toStream(std::iostream &str) ;
    void fromStream(std::istream &str);




   // float markerSize()const{return 1;}


    //returns the ids of the best frames for relocalisation
    vector<uint32_t> relocalizationCandidates(Frame &frame, const std::set<uint32_t> &excludedFrames={});

    //given a  Frame 'curFrame' and a initial position for it, find the keypoints by projecting the mappoints in the refKFrame KeyFrame and its neighbors
    //returns the points of the map visible in the used_frames that are visible in curFrame
    std::vector<cv::DMatch> matchFrameToMapPoints(const std::vector<uint32_t> &used_frames, Frame & curframe, const cv::Mat & pose_f2g, float minDescDist, float maxRepjDist, bool markMapPointsAsVisible, bool useAllPoints=false, std::set<uint32_t> excludedPoints={});


    double globalReprojChi2(const std::vector<uint32_t> &used_frames ={}, std::vector<float > *chi2vector=0,
                                   std::vector<std::pair<uint32_t,uint32_t> > *map_frame_ids=0, bool useKeyPoints=true, bool useMarkers=false) ;



    //updates the normals and other information of the point when its position or the position of the related frames changes
    void updatePointInfo(uint32_t pid);
    void updatePointNormalAndDistances(uint32_t pid);

    void removeBadAssociations(const vector<std::pair<uint32_t,uint32_t>> &BadAssociations,int minNumPtObs);

    //given the frame passed, returns the best possible estimation of the pose using only the markers in the map
    se3 getBestPoseFromValidMarkers(const Frame &frame, const vector<uint32_t> &markersOfFrameToBeUsed, float minErrorRatio);

    template<typename Iterator>
    std::vector<uint32_t> getMapPointsInFrames(Iterator fstart,Iterator fend,const std::set<uint32_t> &excludedPoints={});


private:
    std::mutex IoMutex;//mutex to syncronize mapper and tracker
    std::mutex consitencyMutex;//mutex to syncronize mapper and tracker

    std::vector<cv::Vec4f> getMapPoints4Export( cv::Scalar colorPoints=cv::Scalar(0,0,0),cv::Scalar colorKeyFrames=cv::Scalar(255,0,0),cv::Scalar colorMarkers=cv::Scalar(0,0,255),const set<uint32_t> &specialKeyFrames={},cv::Scalar colorSpecialKeyFrame=cv::Scalar(-1,-1,-1))const;

    std::vector<cv::Vec4f> getPcdPointsLine(const cv::Point3f &a,const cv::Point3f &b,cv::Scalar color,int npoints )const;
    std::vector<cv::Vec4f> getPcdPoints(const vector<cv::Point3f> &mpoints,cv::Scalar color,int npoints=100 )const;
    std::vector<cv::Vec4f>  getMarkerIdPcd(const Marker &marker,  cv::Scalar color )const;

};

template<typename Iterator>
std::vector<uint32_t> Map::getMapPointsInFrames(Iterator fstart, Iterator fend,const std::set<uint32_t> &excludedPoints){

    vector<char> usedpoints(map_points.capacity(),0);
    for(auto f=fstart;f!=fend;f++){
        if (!keyframes.is(*f)) continue;
        const auto &keyframe=keyframes[*f];
        if (keyframe.isBad()) continue;
        for(auto id:keyframe.ids){
            if (id!=std::numeric_limits<uint32_t>::max()){
                assert (id< usedpoints.size());
                usedpoints[id]=1;
            }
        }
    }
    std::vector<uint32_t> usedPointIds;
    usedPointIds.reserve(usedpoints.size());
    for(size_t i=0;i<usedpoints.size();i++)
        if (usedpoints[i]){
            if(map_points.is(i)){
                if (!map_points[i].isBad()){
                    if (excludedPoints.size()==0)
                        usedPointIds.push_back(i);
                    else{
                        if (excludedPoints.count(i)==0)
                            usedPointIds.push_back(i);
                    }
                }
            }
        }

    return usedPointIds;
}


}

#endif

