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
#ifndef TSLAM_SYSTEM_H
#define TSLAM_SYSTEM_H
#include <map>
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include "tslamtypes.h"
#include "markerdetector.h"
#include "imageparams.h"
#include "basictypes/se3.h"
#include "map_types/frame.h"
namespace tslam{

class MapManager;
class MapInitializer;
class Map;
class FrameExtractor;
class   System{
    friend class tslam_Debugger;
public:

    //
    System();
    //
    ~System();

    /**
     * @brief setParams set the required params before processing images
     * @param map a pointer to the map being used. It can be an empty map or an already created one
     * @param params controlling the system behaviuor
     * @param vocabulary optional path to the dictionary BOW. Without it, relocalization is not yet possible.
     */

    void setParams(std::shared_ptr<Map> map, const  tslam::Params &params, const std::string &vocabulary="",std::shared_ptr<MarkerDetector> mdetector=nullptr);

    //Clear this object setting it to intial state. It will remove all map data
    void clear();

    //returns system params
    static Params  & getParams() ;


    /**
     * @brief process the input image and returns (if possible) the camera location
     * @param in_image input monocular image
     * @param ip parameters of the camera being used
     * @param frameseq_idx  number identifing the image. It is normally the sequence id
     * @param depth optional depth image if using a RGBD camera
     * @param R_image optional monocular image if using a stereo camera
     * @return return a 4x4 transform matrix indicating the transform from the global reference system to the current camera reference system
     */
    cv::Mat process(  cv::Mat &in_image,const ImageParams &ip,uint32_t frameseq_idx, const cv::Mat & depth=cv::Mat(), const cv::Mat &R_image=cv::Mat());


    //Reset the current frame pose. Use it to start tracking in a known map
     void resetTracker();

    //sets the system mode
    void setMode(MODES mode);

    //sets the system in lost mode
   // void resetCurrentPose();

    //returns the number of the last processed framed
    uint32_t getLastProcessedFrame()const;


    // Saves the current state of the system to a file so that it can be recovered later. It is only safe if the system is in sequential mode
    void saveToFile(std::string filepath);
    //Loads the state of the system from a file
    void readFromFile(std::string filepath);

    //returns an string that identifies the system state. It is like a md5 sum the system state
    std::string getSignatureStr()const;

    //only for debugging pourposes perform the global optimization
    void globalOptimization();

    //waits for all threads to finish
    void waitForFinished();


    //returns the index of the current keyframe
    uint32_t getCurrentKeyFrameIndex();


    //returns a pointer to the map being used
    std::shared_ptr<Map> getMap();



    //for internal use
    cv::Mat process(const Frame &frame) ;

private:
    //obfuscate start
    friend class DebugTest;
    pair<cv::Mat,cv::Mat> preprocessDataUnlicensed(const cv::Mat &in_image, ImageParams &ip,const     cv::Mat &R_image );

    uint64_t getSignature(bool print=false)const;
    void  createFrameExtractor();

    //! \brief Estimate current pose using either aruco or the database
    //! pose_out is invalid if the pose is not correcly estimated. If correct estimation, the currentKeyFrame is also indicated.
    //! if keyFrame==-1 value, then a new keyframe should be added
    bool relocalize(Frame &f, se3 &pose_f2g_out ) ;
    struct kp_reloc_solution{
        se3 pose;
        int nmatches=0;
        std::vector<uint32_t> ids;
        std::vector<cv::DMatch> matches;
    };
    bool relocalize_withkeypoints( Frame &f,se3 &pose_f2g_out, const std::set<uint32_t> &excluded={} );
    std::vector<kp_reloc_solution>  relocalization_withkeypoints_(Frame &curFrame, se3 &pose_f2g_out , const std::set<uint32_t> &excluded={});
    bool relocalize_withmarkers(Frame &f, se3 &pose_f2g_out) ;
    void drawMatchesAndMarkersInInputImage(cv::Mat &image,float inv_ScaleFactor )const;//return a drawable image from last process



    // DEBUG
    //converts a uint64 to a string for visualization in debug mode
    string sigtostring(uint64_t) const;

    //Pose estimation
    uint32_t getBestReferenceFrame(const Frame &curKeyFrame,  const se3 &curPose_f2g);

    cv::Mat getPoseFromMarkersInMap(const Frame &frame);
    // bool initialize_depth(const cv::Mat &in_image, const cv::Mat & depth, const ImageParams ip, uint32_t frameseq_idx);


    bool initialize(Frame &f2);
    bool initialize_stereo(Frame &frame);
    bool initialize_monocular(Frame &f2) ;
    se3 track(Frame &f, se3 lastKnownPose);



    std::vector<cv::DMatch> matchMapPtsOnPrevFrame(Frame & curframe, Frame &prev_frame,  float minDescDist, float maxReprjDist);


    void putText(cv::Mat &im,string text,cv::Point p );


    static Params _params;
    std::shared_ptr<Map> TheMap;
    std::shared_ptr<FrameExtractor> fextractor;
    std::shared_ptr<MarkerDetector> marker_detector;
    //used internally
    Frame _cFrame,_prevFrame;//current and previous Frame
    std::shared_ptr<MapInitializer> map_initializer;
    bool isInitialized=false;
    se3 _curPose_f2g;//current pose
    int64_t _curKFRef=-1;//current reference key frame
    STATE currentState=STATE_LOST;
    MODES currentMode=MODE_SLAM;
    std::shared_ptr<MapManager> TheMapManager;
    cv::Mat velocity;
    uint64_t totalNFramesProcessed=0;//not saved
    int64_t lastKFReloc=-1;


    //obfuscate end

};
}

#endif // BASIC_TYPES_H
