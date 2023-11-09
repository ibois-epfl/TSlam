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
#ifndef _MAPINITIALIZER_H
#define _MAPINITIALIZER_H

#include <opencv2/opencv.hpp>
#include <utility>
#include "map.h"
#include "framematcher.h"
namespace tslam
{

/** Class to Initialize the map using either  keypoints or markers
*/
 class MapInitializer
{
    typedef std::pair<int,int> Match;
    Frame _refFrame;
     int _nAttempts=0;
    FrameMatcher  fmatcher;

public:
    enum MODE {ARUCO,KEYPOINTS,BOTH,NONE};

    struct Params    {
        float minDistance=0.1;//expected distance between views
        float minDescDistance=-1;//minimum distance between descriptors
        float markerSize=0;
        float minNumMatches=50;
        float aruco_minerrratio_valid=2.5;
        bool allowArucoOneFrame=false;
        float max_makr_rep_err=2.5;
        float nn_match_ratio=0.8;
        MODE mode=BOTH;
        int maxAttemptsRestart=150;
    };


    Params _params;

 public:


    // Fix the reference frame
    MapInitializer( float sigma = 1.0, int iterations = 200);
    //sets the required params for internal use and resets data
    void setParams(Params &p);
    void reset();



    bool process(const Frame &f, std::shared_ptr<Map> map);


    //Sets the reference frame
    void setReferenceFrame(const Frame &frame);
    //checks if the frame passed can be used to initialize with the second one
    bool initialize(const Frame &frame2, std::shared_ptr<Map> map);




private:
    //given two frames with initial matches, it does the following
  //a) computes the rt using ...
  bool initialize_(  const Frame &frame2,   std::shared_ptr<Map> map );
    //obfuscate start

   vector<cv::DMatch> kfmatches;
    vector<cv::Point3f> matches_3d;


    bool  aruco_one_frame_initialize(const Frame &frame,std::shared_ptr<Map> map);


    //receives a new current frame and the matches between reference and current frame
    //returns the rt matrix between the reference frame and current one.
    //if returns a non-empty matrix, the initialization has been done.
    //If Rt is computed from markers, the value minDistance is employed to determine if there is enough distance between the
    //views to accept the result
    std::pair<cv::Mat,MODE> computeRt(const Frame &frame1, const Frame &frame2,  vector<cv::DMatch> &matches,float minDistance=0);


    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    bool getRtFromMatches(const cv::Mat &CamMatrix,const std::vector<cv::KeyPoint> & ReferenceFrame,const std::vector<cv::KeyPoint> &cur_frame,
                      vector<cv::DMatch> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);



    void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
    void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);

    cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
    cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

    float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);

    float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

    bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

    void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

    int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);


    // Keypoints from Reference Frame (Frame 1)
    vector<cv::KeyPoint> mvKeys1;

    // Keypoints from Current Frame (Frame 2)
    vector<cv::KeyPoint> mvKeys2;

    // Current Matches from Reference to Current
    vector<Match> mvMatches12;

    // Calibration
    cv::Mat mK;

    // Standard Deviation and Variance
    float mSigma, mSigma2;

    // Ransac max iterations
    int mMaxIterations;

    // Ransac sets
    vector<vector<size_t> > mvSets;   

      map<uint32_t,se3>  _marker_se3;
     //obfuscate end



 };

}

#endif // INITIALIZER_H
