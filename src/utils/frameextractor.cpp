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

#include <tbb/task_group.h>


namespace tslam {


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
    _tslamParams.toStream(str);
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
    _tslamParams.fromStream(str);
}

FrameExtractor::FrameExtractor(){
}


void FrameExtractor::setParams(std::shared_ptr<Feature2DSerializable> feature_detector,   const Params &params, std::shared_ptr<tslam::MarkerDetector> mdetector){
    _fdetector=feature_detector;
    if(!mdetector)throw  std::runtime_error("FrameExtractor::setParams invalid marker detector");
    _mdetector=mdetector;

    _tslamParams=params;
    _detectMarkers=params.detectMarkers;
    _detectKeyPoints=params.detectKeyPoints;
    _markerSize=params.aruco_markerSize;
    _featParams=Feature2DSerializable::FeatParams(params.maxFeatures,params.nOctaveLevels,params.scaleFactor, params.nthreads_feature_detector);
    _maxDescDist=params.maxDescDistance;
}
void FrameExtractor::setSensitivity(float v){
    if(_fdetector)
        _fdetector->setSensitivity(v);
}

float  FrameExtractor::getSensitivity(){
    if(!_fdetector)throw std::runtime_error(string(__PRETTY_FUNCTION__)+"Should not call this function since the class is not initialized");
    return _fdetector->getSensitivity();
}


void FrameExtractor::processStereo(const cv::Mat &LeftRect, const cv::Mat &RightRect, const ImageParams &ip, Frame &frame, uint32_t frameseq_idx)
{
    assert(ip.bl>0);

    preprocessImages(_tslamParams.kptImageScaleFactor,LeftRect,ip,RightRect );
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

    preprocessImages(_tslamParams.kptImageScaleFactor,  image,ip);
    extractFrame(InputImages[0],frame,frameseq_idx);
}

void FrameExtractor::preprocessImages(float scaleFactor,const cv::Mat &im1,const ImageParams &ip,const cv::Mat &im2){

    assert(im1.size()==ip.CamSize) ;
    assert(im2.empty() || ( im2.size()==ip.CamSize));
    int nI=1;
    if(! im2.empty())
        nI++;
    InputImages.resize(nI);
    if( im1.channels()==3)
        cv::cvtColor(im1,InputImages[0].im_org,CV_BGR2GRAY);
    else    InputImages[0].im_org=im1;
    InputImages[0].ip_org=ip;
    if(! im2.empty())
    {
        if( im2.channels()==3)
            cv::cvtColor(im2,InputImages[1].im_org,CV_BGR2GRAY);
        else    InputImages[1].im_org=im2;
        InputImages[1].ip_org=ip;
    }

    //    int kpMinConfSugg=18053.0184 *3.141516 * (2*5+3);

    //    kpMinConfSugg=std::numeric_limits<int>::max();
    if(fabs(1-scaleFactor)>1e-3    ){
        cv::Size ns(im1.cols*scaleFactor,im1.rows*scaleFactor);
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
            cv::resize(InputImages[1].im_org,InputImages[1].im_resized,ns);
            InputImages[1].ip_resized=InputImages[2].ip_org;
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
    __TSLAM_ADDTIMER__;

    tbb::task_group group;

    std::unique_ptr<std::thread> kp_thread, aruco_thread;

    vector<cv::KeyPoint> frame_kpts;
    if(_detectKeyPoints){
        group.run([&]{
            _fdetector->detectAndCompute(InputImages[0].im_resized,cv::Mat(),frame_kpts,frame.desc,_featParams);
            frame.KpDescType=_fdetector->getDescriptorType();
        });
    }

    if (_detectMarkers){
        group.run([&]{
            auto markers=_mdetector->detect(Iinfo.im_org);
            for(const auto&m:markers){
                tslam::MarkerObservation uslm_marker;
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
        });
    }

    group.wait();

    if (debug::Debug::getLevel()>=100|| _tslamParams.saveImageInMap){
            //encode
            std::vector<uchar> buf;
            cv::imencode(".jpg",Iinfo.im_org,buf,{cv::IMWRITE_JPEG_QUALITY,90});
            frame.jpeg_buffer.create(1,buf.size(),CV_8UC1);
            mempcpy(frame.jpeg_buffer.ptr<uchar>(0),&buf[0],buf.size());
    }


    //remove keypoints into markers??

    __TSLAM_TIMER_EVENT__("Keypoint/Frames detection");


    //Create the scale factors vector
    frame.scaleFactors.resize(_fdetector->getParams().nOctaveLevels);
    double sf=_fdetector->getParams().scaleFactor;
    frame.scaleFactors[0]=1;
    for(size_t i=1;i<frame.scaleFactors.size();i++)
        frame.scaleFactors[i]=frame.scaleFactors[i-1]*sf;

    __TSLAM_TIMER_EVENT__("remove from markers");

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

    __TSLAM_TIMER_EVENT__("undistort");
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
