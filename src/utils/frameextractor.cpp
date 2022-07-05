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
#include "frameextractor.h"
#include <thread>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "basictypes/timers.h"
#include "basictypes/misc.h"
#include "optimization/ippe.h"
#include "map_types/mappoint.h"
#include <aruco/markerdetector.h>
#include "basictypes/osadapter.h"
#include "basictypes/cvversioning.h"
#include "xflann/xflann.h"
#include "framematcher.h"


namespace reslam {


void FrameExtractor::toStream(std::ostream &str)const
{
    uint64_t sig=1923123;
    str.write((char*)&sig,sizeof(sig));
    _fdetector->toStream(str);
    str.write((char*)&_counter,sizeof(_counter));
    str.write((char*)&_removeFromMarkers,sizeof(_removeFromMarkers));
    str.write((char*)&_detectMarkers,sizeof(_detectMarkers));
    str.write((char*)&_detectKeyPoints,sizeof(_detectKeyPoints));
    str.write((char*)&_markerSize,sizeof(_markerSize));

    str.write((char*)&_featParams,sizeof(_featParams));
    str.write((char*)&_maxDescDist,sizeof(_maxDescDist));
  //  _mdetector->toStream(str);
    _ucoslamParams.toStream(str);
}

void FrameExtractor::fromStream(std::istream &str){
    uint64_t sig=1923123;
    str.read((char*)&sig,sizeof(sig));
    if (sig!=1923123) throw std::runtime_error(string(__PRETTY_FUNCTION__)+"invalid signature");
    _fdetector= Feature2DSerializable::fromStream(str);
    str.read((char*)&_counter,sizeof(_counter));
    str.read((char*)&_removeFromMarkers,sizeof(_removeFromMarkers));
    str.read((char*)&_detectMarkers,sizeof(_detectMarkers));
    str.read((char*)&_detectKeyPoints,sizeof(_detectKeyPoints));
    str.read((char*)&_markerSize,sizeof(_markerSize));
    str.read((char*)&_featParams,sizeof(_featParams));
    str.read((char*)&_maxDescDist,sizeof(_maxDescDist));
//    _mdetector->fromStream(str);
    _ucoslamParams.fromStream(str);
}

FrameExtractor::FrameExtractor(){
}


void FrameExtractor::setParams(std::shared_ptr<Feature2DSerializable> feature_detector,   const Params &params, std::shared_ptr<reslam::MarkerDetector> mdetector){
    _fdetector=feature_detector;
    if(!mdetector)throw  std::runtime_error("FrameExtractor::setParams invalid marker detector");
    _mdetector=mdetector;

    _ucoslamParams=params;
    _detectMarkers=params.detectMarkers;
    _detectKeyPoints=params.detectKeyPoints;
    _markerSize=params.aruco_markerSize;
    _maxDescDist=params.maxDescDistance;
}

void FrameExtractor::initFeatParams(const ImageParams &ip)
{
    if(_featParams.nOctaveLevels>0) return;

    //Take camera id with minimun focal. With cameras stero with same focal, it is no necessary
    int _idFocal=0;
    for(int i=1; i<ip.arraySize(); i++)
    {
        if(ip.fx(i)<ip.fx(_idFocal))
            _idFocal= i;
    }


    float maxFocalLength;
    if(_ucoslamParams.maxOctave!=-1)
        maxFocalLength =  std::min(_ucoslamParams.minFocalLength * pow(_ucoslamParams.scaleFactor, _ucoslamParams.maxOctave), (double)ip.fx(_idFocal));
    else
        maxFocalLength = ip.fx(_idFocal) * _ucoslamParams.kptImageScaleFactor;



    //Get nOctaveLevels and maxFeatures using scaleFactor param
    int _nOctaveLevels=0;
    int _maxFeatures=0;


    float focal=_ucoslamParams.minFocalLength;
    while(focal<=maxFocalLength)
    {
        //Total features
        _maxFeatures+=_ucoslamParams.featuresFirstLevel*pow(_ucoslamParams.scaleFactor, _nOctaveLevels);
        _nOctaveLevels++;
        focal*=_ucoslamParams.scaleFactor;
    }
    //Scale the max number of features
    _ucoslamParams.maxFeatures*=_ucoslamParams.featuresFactor;

    std::cout << "Original focal length:" << ip.fx(_idFocal)<<std::endl;
    std::cout << "Scaled focal length:" << _ucoslamParams.minFocalLength * pow(_ucoslamParams.scaleFactor,_nOctaveLevels-1)<<std::endl;
    std::cout << "Max features: "<<_maxFeatures<<std::endl;
    std::cout << "Total scales: "<<_nOctaveLevels<<std::endl;

    _ucoslamParams.maxOctave=_nOctaveLevels-1;

    _featParams = Feature2DSerializable::FeatParams(_maxFeatures,_nOctaveLevels,_ucoslamParams.scaleFactor, _ucoslamParams.nthreads_feature_detector);
}

void FrameExtractor::setSensitivity(float v){
    if(_fdetector)
        _fdetector->setSensitivity(v);
}

float  FrameExtractor::getSensitivity(){
    if(!_fdetector)throw std::runtime_error(string(__PRETTY_FUNCTION__)+"Should not call this function since the class is not initialized");
    return _fdetector->getSensitivity();
}


void FrameExtractor::processArray(const vector<cv::Mat> &images, const ImageParams &ArrayCamParams , Frame &frame, uint32_t frameseq_idx, const std::shared_ptr<MapInitializer> map_init)
{
    std::vector<cv::DMatch> matches;

    assert(ArrayCamParams.isArray());
    preprocessArrayImages(_ucoslamParams.kptImageScaleFactor,images,ArrayCamParams);
    extractFrame(InputImages[0],frame,frameseq_idx);

    frame.depth.resize(frame.und_kpts.size());
    for(size_t i=0;i<frame.depth.size();i++) frame.depth[i]=0;

    vector<cv::KeyPoint> trainKpts;
    cv::Mat trainDesc;
    _fdetector->detectAndCompute(InputImages[1].im_resized, cv::Mat(),trainKpts,trainDesc,_featParams);

    //Undistort train kpoints
    vector<cv::Point2f> und_trainPnts; und_trainPnts.reserve(trainKpts.size());
    for(auto p:trainKpts) und_trainPnts.push_back(p.pt);
    InputImages[0].ip_resized.undistortPoints(und_trainPnts,nullptr,1);

    vector<cv::KeyPoint> und_trainKpts;
    und_trainKpts = trainKpts;
    for ( size_t i=0; i<trainKpts.size(); i++ )
        und_trainKpts[i].pt=und_trainPnts[i];

    map_init->fmatcher.setParams(frame,FrameMatcher::MODE_ALL,map_init->_params.minDescDistance,map_init->_params.nn_match_ratio,true);
    //Fundamental matrix
    cv::Mat Fund = map_init->fmatcher.getFund12( InputImages[0].ip_resized.CameraMatrix,InputImages[0].ip_resized.arrayCamMatrix[0],
            getRTMatrix(InputImages[0].ip_resized.arrayRvec[0], InputImages[0].ip_resized.arrayTvec[0]).inv());

    getMatches(matches, frame, und_trainKpts, Fund, trainDesc, 0);

    filter_ambiguous_train(matches);

    std::vector<cv::KeyPoint> r_kpts;
    std::vector<cv::KeyPoint> l_kpts;
    for(size_t m=0; m<matches.size(); m++)
    {
        l_kpts.push_back(frame.und_kpts[matches[m].queryIdx]);
        r_kpts.push_back(und_trainKpts[matches[m].trainIdx]);
    }

    std::vector<cv::Point3f> p3d;
    std::vector<bool> vbGood;
    int goods = triangulate_(getRTMatrix(ArrayCamParams.arrayRvec[0],ArrayCamParams.arrayTvec[0]).inv(), l_kpts, r_kpts,
            InputImages[0].ip_resized.CameraMatrix, InputImages[0].ip_resized.arrayCamMatrix[0],
            p3d,frame.scaleFactors,vbGood);


    for(uint i=0; i<vbGood.size(); i++)
        if(vbGood[i])
            frame.depth[matches[i].queryIdx]=p3d[i].z;

/////////**
///// Draw matches
/////////**
//    std::vector<cv::DMatch> g_matches;
//    g_matches.resize(goods);
//    int k=0;
//    for(uint i=0; i<vbGood.size(); i++)
//    {
//        if(vbGood[i]){
//            g_matches[k] = matches[i];
//            k++;
//        }
//    }

//    cv::Mat img_match2, lCopy, rCopy;

//    cv::undistort(InputImages[0].im_resized, lCopy, InputImages[0].ip_resized.CameraMatrix,
//            InputImages[0].ip_resized.Distorsion, cv::noArray());
//    cv::undistort(InputImages[1].im_resized, rCopy, InputImages[0].ip_resized.arrayCamMatrix[0],
//            InputImages[0].ip_resized.arrayDistorsion[0], cv::noArray());

//    drawMatches(lCopy, frame.und_kpts , rCopy, und_trainKpts, g_matches, img_match2);
//    cv::resize(img_match2, img_match2, cv::Size(1800,900));
//    imshow("Matches2", img_match2);
//    cv::waitKey();
///////**

    _debug_msg_(" num matches:"<<matches.size());
}

void FrameExtractor::getMatches(std::vector<cv::DMatch>& matches,  const Frame &frame, const std::vector<cv::KeyPoint> &trainKpts,
                                const cv::Mat &Fund, const cv::Mat &trainDesc, int t)
{
    cv::Mat indices,distances;
    if(t==0)
    {
        int maxSearch=16;
        int nn=10;
        xflann::Index trainIndex;
        trainIndex.build(trainDesc, xflann::HKMeansParams(32,0));
        trainIndex.search(frame.desc,nn,indices,distances ,xflann::KnnSearchParams(maxSearch,false));
        if ( distances.type()==CV_32S)
            distances.convertTo(distances,CV_32F);
    }


    vector<float> scaleFactor2;
    for(int i=frame.scaleFactors.size()-1; i>=0; i-- )
        scaleFactor2.push_back(frame.scaleFactors[i]*frame.scaleFactors[i]);
    int nval=0;

    for(size_t queryIdx=0; queryIdx<frame.und_kpts.size(); queryIdx++)
    {
        //Epipolar line
        float ep_a = frame.und_kpts[queryIdx].pt.x*Fund.at<float>(0,0)+frame.und_kpts[queryIdx].pt.y*Fund.at<float>(1,0)+Fund.at<float>(2,0);
        float ep_b = frame.und_kpts[queryIdx].pt.x*Fund.at<float>(0,1)+frame.und_kpts[queryIdx].pt.y*Fund.at<float>(1,1)+Fund.at<float>(2,1);
        float ep_c = frame.und_kpts[queryIdx].pt.x*Fund.at<float>(0,2)+frame.und_kpts[queryIdx].pt.y*Fund.at<float>(1,2)+Fund.at<float>(2,2);

////////Draw
//        cv::Mat leftImg, rightImg;
//        cv::undistort(InputImages[0].im_resized, leftImg, InputImages[0].ip_resized.CameraMatrix,
//                InputImages[0].ip_resized.Distorsion, cv::noArray());
//        cv::undistort(InputImages[1].im_resized, rightImg, InputImages[0].ip_resized.arrayCamMatrix[0],
//                InputImages[0].ip_resized.arrayDistorsion[0], cv::noArray());
//        cv::cvtColor(leftImg,leftImg,cv::COLOR_GRAY2BGR);
//        cv::cvtColor(rightImg,rightImg,cv::COLOR_GRAY2BGR);
////        cv::drawKeypoints(leftImg,frame.und_kpts,leftImg);
////        cv::drawKeypoints(rightImg,trainKpts,rightImg);
//        cv::circle(leftImg, frame.und_kpts[queryIdx].pt, 4, cv::Scalar(0,0,255),-1);
//////////

        float _bestDist1=std::numeric_limits<float>::max(),_bestDist2=std::numeric_limits<float>::max();
        int64_t _bestTrain=-1, _bestQuery=-1, _octaveBest2=-1;

        if(t==0)
        {
            for(int j=0; j<indices.cols; j++)
            {

                int trainIdx= indices.at<int>(queryIdx,j);

                ////////Draw: epipolar & right point
//                cv::Point2f p1 = cv::Point2f(0,-ep_c/ep_b);
//                float xx=InputImages[1].im_resized.cols-1;
//                cv::Point2f p2 = cv::Point2f(xx, (-ep_a*xx-ep_c)/ep_b);
//                cv::line(rightImg, p1, p2, cv::Scalar(0,0,255), 1);
//                cv::circle(rightImg, trainKpts[trainIdx].pt, 4, cv::Scalar(0,0,255),-1);
//                cv::imshow("Left",leftImg);
//                cv::imshow("Right",rightImg);
//                cv::waitKey();
                ////////

                if(std::abs(frame.und_kpts[queryIdx].octave - trainKpts[trainIdx].octave) > 1) continue;

                float d = epipolarLineSqDist(frame.und_kpts[queryIdx].pt, trainKpts[trainIdx].pt, Fund);
                if (d >=3.84*scaleFactor2[frame.und_kpts[queryIdx].octave])continue;



                if ( distances.at<float>(queryIdx,j)>_maxDescDist) continue;




                if (distances.at<float>(queryIdx,j)<_bestDist2)
                {
                    if (distances.at<float>(queryIdx,j)<_bestDist1)
                    {
                        nval++;

                        _bestDist1=distances.at<float>(queryIdx,j);
                        _bestQuery = queryIdx;
                        _bestTrain=trainIdx;
                    }
                    else
                    {
                        _bestDist2=distances.at<float>(queryIdx,j);
                        _octaveBest2=trainKpts[trainIdx].octave;
                    }
                }
            }
        }
        else if(t==1){
            //Search good matches in the other img
            for(size_t trainIdx=0; trainIdx<trainKpts.size(); trainIdx++)
            {

                float dst = fabs(ep_a*trainKpts[trainIdx].pt.x + ep_b*trainKpts[trainIdx].pt.y + ep_c) / sqrt(pow(ep_a,2) + pow(ep_b,2));

                if (dst >=3.84*scaleFactor2[frame.und_kpts[queryIdx].octave]) continue;

//////////Draw: epipolar & right point
//                cv::Point2f p1 = cv::Point2f(0,-ep_c/ep_b);
//                float xx=InputImages[1].im_resized.cols-1;
//                cv::Point2f p2 = cv::Point2f(xx, (-ep_a*xx-ep_c)/ep_b);
//                cv::line(rightImg, p1, p2, cv::Scalar(0,0,255), 1);

//                cv::circle(rightImg, trainKpts[trainIdx].pt, 4, cv::Scalar(0,0,255),-1);
//////////

                if(std::abs(frame.und_kpts[queryIdx].octave - trainKpts[trainIdx].octave) > 1) continue;

                auto distDesc=MapPoint::getDescDistance(frame.desc,queryIdx,trainDesc,trainIdx);
                if ( distDesc>_maxDescDist) continue;

                if (distDesc<_bestDist2)
                {
                    if (distDesc<_bestDist1)
                    {
                        _bestDist1=distDesc;
                        _bestQuery = queryIdx;
                        _bestTrain=trainIdx;
                    }
                    else
                    {
                        _bestDist2=distDesc;
                        _octaveBest2=trainKpts[trainIdx].octave;
                    }
                }
            }
        }
        /////////
//        cv::imshow("Left",leftImg);
//        cv::imshow("Right",rightImg);
//        cv::waitKey();
        /////////



        if ( _bestQuery!=-1){
            if(!( _octaveBest2==frame.und_kpts[_bestQuery].octave &&  _bestDist1 > _bestDist2*0.8f  ))
            {
                cv::DMatch  match;
                match.queryIdx=_bestQuery;
                match.trainIdx=_bestTrain;
                match.distance=_bestDist1;
                matches.push_back(match);

/////////
//                std::vector<cv::DMatch> test_match;
//                test_match.push_back(match);
//                cv::Mat rightCopy;
//                cv::undistort(InputImages[1].im_resized, rightCopy, InputImages[0].ip_resized.arrayCamMatrix[0],
//                        InputImages[0].ip_resized.arrayDistorsion[0], cv::noArray());

//                //Draw epipolar
//                cv::Point2f undist_p1 = cv::Point2f(0,-ep_c/ep_b);
//                float xx=InputImages[1].im_resized.cols-1;
//                cv::Point2f undist_p2 = cv::Point2f(xx, (-ep_a*xx-ep_c)/ep_b);
//                cv::line(rightCopy, undist_p1, undist_p2, cv::Scalar(0,0,255), 1);

//                cv::Mat matchesImg;
//                drawMatches(InputImages[0].im_resized,frame.und_kpts , rightCopy,
//                        trainKpts, test_match, matchesImg);

//                cv::resize(matchesImg, matchesImg, cv::Size(1800,900));
//                imshow("Matches Img", matchesImg);
//                cv::waitKey();
///////
            }
        }
    }
}

void FrameExtractor::processStereo(const cv::Mat &LeftRect, const cv::Mat &RightRect, const ImageParams &ip, Frame &frame, uint32_t frameseq_idx)
{
    assert(ip.bl>0);

    preprocessImages(_ucoslamParams.kptImageScaleFactor,LeftRect,ip,RightRect );
    extractFrame(InputImages[0],frame,frameseq_idx);

    frame.depth.resize(frame.und_kpts.size());
    for(size_t i=0;i<frame.depth.size();i++) frame.depth[i]=0;


    vector<cv::KeyPoint> rightKPs;
    cv::Mat rightdesc;
    _fdetector->detectAndCompute(InputImages[1].im_resized,cv::Mat(),rightKPs,rightdesc,_featParams);

    vector<vector<int>> y_keypoints(InputImages[1].im_resized.rows);
    for(size_t i=0; i<rightKPs.size(); i++){
        double position_radius=0;//rightKPs[i].size/100;
        double y=rightKPs[i].pt.y;
        int min_y=std::max(0,int(std::round(y-position_radius)));
        int max_y=std::min(InputImages[1].im_resized.rows-1,int(std::round(y+position_radius)));
        for(int y=min_y;y<=max_y;y++)
            y_keypoints[y].push_back(i);
    }

    //vector<cv::DMatch> matches;
    int num_matches=0;
    //#pragma omp parallel for
    for(size_t i=0; i<frame.und_kpts.size(); i++){
        //get the vector of right keypoints associated with the same y as the right keypoint
        int y=std::round(frame.und_kpts[i].pt.y);
        vector<int> &rkpts=y_keypoints[y];
        //get a copy of the left keypoint
        //cv::KeyPoint shifted_lkp=frame.kpts[i];
        //vars to keep the best match on the right
        int best_rkpt=-1;
        double best_rkpt_sad=std::numeric_limits<double>::max();
        //go through the vector
        //std::cout<<rkpts.size()<<std::endl;

        for(size_t j=0; j<rkpts.size(); j++){
            int rkpt_index=rkpts[j];
            if(rightKPs[rkpt_index].pt.x > frame.und_kpts[i].pt.x || std::abs(rightKPs[rkpt_index].octave-frame.und_kpts[i].octave)>1 )
                continue;
            //shifted_lkp.pt.x=rightKPs[rkpt_index].pt.x;
            //if(cv::KeyPoint::overlap(rightKPs[rkpt_index],shifted_lkp)>0.90){

                auto dist=MapPoint::getDescDistance(frame.desc,i,rightdesc,rkpt_index);

                if(dist< _maxDescDist){
                    //cout<<SAD<<endl;
                    if(dist<best_rkpt_sad){
                        best_rkpt_sad=dist;
                        best_rkpt=rkpt_index;
                    }
                }
            //}
        }

        if(best_rkpt != -1){
            //matches.push_back(cv::DMatch(i,best_rkpt,best_rkpt_sad));
            //calculate the depth

            //refine the position of the match on the right

            int patch_size=7;
            int half_patch_size=patch_size/2;

            int left_patch_x=std::round(frame.und_kpts[i].pt.x);
            int left_patch_y=std::round(frame.und_kpts[i].pt.y);

            if(left_patch_x<half_patch_size || left_patch_x+half_patch_size>=LeftRect.cols)
                continue;

            if(left_patch_y<half_patch_size || left_patch_y+half_patch_size>=LeftRect.rows)
                continue;

            int right_patch_x=std::round(rightKPs[best_rkpt].pt.x);
            int right_patch_y=std::round(rightKPs[best_rkpt].pt.y);

            if(right_patch_x<half_patch_size || right_patch_x+half_patch_size>=RightRect.cols)
                continue;

            if(right_patch_y<half_patch_size || right_patch_y+half_patch_size>=RightRect.rows)
                continue;

            int search_bound=7;
            int search_window_size=search_bound*2+1;
            vector<double> correlations(search_window_size);

            cv::Mat left_patch= InputImages[0].im_resized(cv::Range(left_patch_y-half_patch_size,left_patch_y+half_patch_size),cv::Range(left_patch_x-half_patch_size,left_patch_x+half_patch_size));
             //cout<<"num channels:"<<left_patch.channels()<<endl;
            int min_delta=std::max(-search_bound,-right_patch_x);
            int max_delta=std::min(search_bound,RightRect.cols-1-right_patch_x);

            double min_correlation=std::numeric_limits<double>::max();
            int min_c_index=-1;
            for(int delta=min_delta;delta<=max_delta;delta++){
                int c_index=delta+search_bound;
                int rx=right_patch_x+delta;
                cv::Mat right_patch=InputImages[1].im_resized(cv::Range(right_patch_y-half_patch_size,right_patch_y+half_patch_size),cv::Range(rx-half_patch_size,rx+half_patch_size));
                cv::Mat abs_diff;
                cv::absdiff(left_patch,right_patch,abs_diff);
                double correlation=cv::sum(abs_diff)[0];
                if(correlation<min_correlation){
                    min_correlation=correlation;
                    min_c_index=c_index;
                }
                correlations[c_index]=correlation;
            }

            if(min_c_index>min_delta+search_bound && min_c_index<max_delta+search_bound){//if it is not the first and the last value
                //interpolate the extremum
                double f1=correlations[min_c_index-1];
                double f2=correlations[min_c_index];
                double f3=correlations[min_c_index+1];
                double delta_min=0.5*(f1-f3)/(f1+f3-2*f2)+min_c_index-search_bound;
                double optimal_right_pos=rightKPs[best_rkpt].pt.x+delta_min;
                frame.depth[i]=(ip.bl*ip.fx())/(frame.und_kpts[i].pt.x-optimal_right_pos);
                num_matches ++;
            }
        }
    }
    _debug_msg_(" num matches:"<<num_matches);
 }

void FrameExtractor::process_rgbd(const cv::Mat &image, const cv::Mat &depthImage,const ImageParams &ip,Frame &frame, uint32_t frameseq_idx ){
    assert(ip.bl>0);
    assert(depthImage.type()==CV_16UC1);

    preprocessImages(1,image,ip);
    extractFrame(InputImages[0],frame,frameseq_idx);
    //process(image,ip,frame,frameseq_idx);

    frame.depth.resize(frame.und_kpts.size());

    for(size_t i=0;i<frame.depth.size();i++) frame.depth[i]=0;
    //now, add the extra info to the points
    for(size_t i=0;i<frame.kpts.size();i++){
        //convert depth
        if (depthImage.at<uint16_t>(frame.kpts[i])!=0  ){
            frame.depth[i]=depthImage.at<uint16_t>(frame.kpts[i])*ip.rgb_depthscale;
        }
    }
}


void FrameExtractor::process(const cv::Mat &image, const ImageParams &ip,Frame &frame, uint32_t frameseq_idx)
{

    preprocessImages(_ucoslamParams.kptImageScaleFactor,  image,ip);
    extractFrame(InputImages[0],frame,frameseq_idx);
}

void FrameExtractor::preprocessArrayImages(float scaleFactor,const std::vector<cv::Mat> &images,const ImageParams &ip){


    assert(images[0].size()==ip.CamSize) ;
    for(uint i=0; i<ip.multicams_cs.size(); i++)
        assert(images[i+1].size() == ip.multicams_cs[i]);

    InputImages.resize(images.size());
    for(uint i=0; i<images.size(); i++) {
        if( images[i].channels()==3)
            cv::cvtColor( images[i],InputImages[i].im_org,CV_BGR2GRAY);
        else    InputImages[i].im_org= images[i];
    }

    //Image params are shared by all the cameras (InputImages[0])
    InputImages[0].ip_org=ip;
    InputImages[0].ip_resized=ip;

    float newFocalLength = _ucoslamParams.minFocalLength * pow(_ucoslamParams.scaleFactor, _featParams.nOctaveLevels-1);
    for(uint i=0; i<images.size(); i++) {
        float rate = newFocalLength/InputImages[0].ip_org.fx(i);

        if(fabs(1-rate)>1e-3) {
//                    std::cout << "Rate" <<rate << std::endl;
            cv::Size ns(images[i].cols*rate,images[i].rows*rate);
            //ensure the size has zero padding
            if(ns.width%4!=0)
                ns.width+=4-ns.width%4;
            if(ns.height%2!=0)ns.height++;
            cv::resize(InputImages[i].im_org,InputImages[i].im_resized,ns);
            InputImages[0].ip_resized.resize(ns, i);

            InputImages[i].scaleFactor.first= float(ns.width)/float(images[i].cols) ;
            InputImages[i].scaleFactor.second= float(ns.height)/float(images[i].rows) ;
        }
        else{
            InputImages[i].im_resized=InputImages[i].im_org;
            InputImages[i].scaleFactor=std::pair<float,float>(1,1);
        }
    }
}

void enhanceImageBGR(const cv::Mat &imgIn, cv::Mat &imgOut){
    // Start CLAHE Contrast Limited and Adaptive Histogram Equalization

    //Get Intesity image
    cv::Mat Lab_image;
    cvtColor(imgIn, Lab_image, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> Lab_planes(3);
    cv::split(Lab_image, Lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    // clahe->setTilesGridSize(cv::Size(10, 10));
    cv::Mat clahe_L;
    clahe->apply(Lab_planes[0], clahe_L);

    // Merge the color planes back into an Lab image
    clahe_L.copyTo(Lab_planes[0]);
    cv::merge(Lab_planes, Lab_image);

    // convert back to RGB
    cv::cvtColor(Lab_image, imgOut, cv::COLOR_Lab2BGR);
}

void FrameExtractor::preprocessImages(float scaleFactor,const cv::Mat &im1,const ImageParams &ip,const cv::Mat &im2){

    assert(im1.size()==ip.CamSize) ;
    assert(im2.empty() || ( im2.size()==ip.CamSize));

    int nI=1;
    
    if(! im2.empty())
        nI++;
    
    InputImages.resize(nI);
    
    if( im1.channels()==3){
        enhanceImageBGR(im1, InputImages[0].im_org);
        cv::cvtColor(InputImages[0].im_org, InputImages[0].im_org,CV_BGR2GRAY);
    } else {
        InputImages[0].im_org=im1;
    }
        

    InputImages[0].ip_org=ip;

    if(! im2.empty())
    {
        if( im2.channels()==3) {
            enhanceImageBGR(im2, InputImages[1].im_org);
            cv::cvtColor(InputImages[1].im_org, InputImages[1].im_org,CV_BGR2GRAY);
        } else {
            InputImages[1].im_org=im2;
        }
        InputImages[1].ip_org=ip;
    }


    int newFocalLength = _ucoslamParams.minFocalLength * pow(_ucoslamParams.scaleFactor, _featParams.nOctaveLevels-1);
    float rate = newFocalLength/InputImages[0].ip_org.fx();
//    std::cout << "Rate" <<rate << std::endl;

    if(fabs(1-rate)>1e-3){
//    if(fabs(1-rate)>0.1f){
        cv::Size ns(im1.cols*rate,im1.rows*rate);
        //ensure the size has zero padding
        if(ns.width%4!=0)
            ns.width+=4-ns.width%4;
        if(ns.height%2!=0)ns.height++;
        cv::resize(InputImages[0].im_org,InputImages[0].im_resized,ns);
        InputImages[0].ip_resized=InputImages[0].ip_org;
        InputImages[0].ip_resized.resize(ns);

        InputImages[0].scaleFactor.first= float(ns.height)/float(im1.rows) ;
        InputImages[0].scaleFactor.second= float(ns.width)/float(im1.cols) ;

        if(!im2.empty()){
            InputImages[1].im_resized=InputImages[1].im_org;
            cv::resize(InputImages[1].im_org,InputImages[1].im_resized,ns);
            InputImages[1].ip_resized=InputImages[1].ip_org;
            InputImages[1].ip_resized.resize(ns);
            InputImages[1].scaleFactor.first= float(ns.height) /float(im2.rows);
            InputImages[1].scaleFactor.second= float(ns.width)/float(im2.cols) ;
        }
    }
    else{
        for(int i=0;i<InputImages.size();i++){
            InputImages[i].im_resized=InputImages[i].im_org;
            InputImages[i].ip_resized=InputImages[i].ip_org;
            InputImages[i].scaleFactor=std::pair<float,float>(1,1);
        }
    }
 }


void FrameExtractor::extractFrame(const ImgInfo &Iinfo,   Frame &frame, uint32_t frameseq_idx){

    frame.clear();
    __UCOSLAM_ADDTIMER__

    vector<cv::KeyPoint> frame_kpts;
    std::thread kp_thread( [&]{
        if(_detectKeyPoints){
            _fdetector->detectAndCompute(InputImages[0].im_resized, cv::Mat(),frame_kpts,frame.desc,_featParams);

            frame.KpDescType=_fdetector->getDescriptorType();
        }
    });

    // TODO: change marker detector
    std::thread aruco_thread( [&]{
        if (_detectMarkers){
            auto markers=_mdetector->detect(Iinfo.im_org);
            for(const auto&m:markers){
                reslam::MarkerObservation uslm_marker;
                uslm_marker.id=m.id;
                uslm_marker.points3d=m.points3d;
                uslm_marker.corners=m.corners;
                uslm_marker.dict_info=m.info;
                auto sols=IPPE::solvePnP_(m.points3d ,m.corners, Iinfo.ip_org.CameraMatrix,Iinfo.ip_org.Distorsion);
                for(int a=0;a<2;a++){
                    uslm_marker.poses.errs[a]=sols[a].second;
                    uslm_marker.poses.sols[a]=sols[a].first.clone();
                }
                uslm_marker.poses.err_ratio=sols[1].second/sols[0].second;
                for(auto &c:uslm_marker.corners) {
                    c.x*=Iinfo.scaleFactor.first;//scale
                    c.y*=Iinfo.scaleFactor.second;//scale
                }
                frame.markers.push_back(uslm_marker);
            }
        }
    }
    );
    kp_thread.join();
    aruco_thread.join();


    if (debug::Debug::getLevel()>=100|| _ucoslamParams.saveImageInMap){
            //encode
            std::vector<uchar> buf;
            cv::imencode(".jpg",Iinfo.im_org,buf,{cv::IMWRITE_JPEG_QUALITY,90});
            frame.jpeg_buffer.create(1,buf.size(),CV_8UC1);
            memcpy(frame.jpeg_buffer.ptr<uchar>(0),&buf[0],buf.size());
    }


    //remove keypoints into markers??

    __UCOSLAM_TIMER_EVENT__("Keypoint/Frames detection");

    //Create the scale factors vector
    frame.scaleFactors.resize(_fdetector->getParams().nOctaveLevels);
    double sf=_fdetector->getParams().scaleFactor;
    frame.scaleFactors[0]=1;
    for(size_t i=1;i<frame.scaleFactors.size();i++)
        frame.scaleFactors[i]=frame.scaleFactors[i-1]*sf;

    __UCOSLAM_TIMER_EVENT__("remove from markers");

    //remove distortion

    if (frame_kpts.size()>0){
        vector<cv::Point2f> pin;pin.reserve(frame_kpts.size());
        for(auto p:frame_kpts) pin.push_back(p.pt);
        InputImages[0].ip_resized.undistortPoints(pin );

        frame.und_kpts=frame_kpts;
        frame.kpts.resize(frame_kpts.size());
        for ( size_t i=0; i<frame_kpts.size(); i++ ){
            frame.kpts[i]=frame_kpts[i].pt;
            frame.und_kpts[i].pt=pin[i];
        }
    }

    __UCOSLAM_TIMER_EVENT__("undistort");
    //remove distortion of the marker corners if any
    for(auto &m:frame.markers){
        m.und_corners=m.corners;
        InputImages[0].ip_resized.undistortPoints(m.und_corners);
    }


    frame.flags.resize(frame.und_kpts.size());
    for(auto &v:frame.flags) v.reset();

    frame.ids.resize(frame.und_kpts.size());
    //set the keypoint ids vector to invalid
    uint32_t mval=std::numeric_limits<uint32_t>::max();
    for(auto &ids:frame.ids) ids=mval;
    //create the grid for fast access


    frame.idx=std::numeric_limits<uint32_t>::max();
    frame.fseq_idx=frameseq_idx;
    frame.imageParams=InputImages[0].ip_resized;

    frame.create_kdtree();//last thing

    //set image projection limits
    frame.minXY=cv::Point2f(0,0);
    frame.maxXY=cv::Point2f(frame.imageParams.CamSize.width,frame.imageParams.CamSize.height);
    if(frame.imageParams.Distorsion.total()!=0){
        //take the points and remove distortion
        vector<cv::Point2f> corners={frame.minXY,frame.maxXY};
        frame.imageParams.undistortPoints(corners);
        frame.minXY=corners[0];
        frame.maxXY=corners[1];
    }


}

}
