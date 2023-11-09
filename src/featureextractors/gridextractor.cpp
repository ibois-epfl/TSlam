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
#include "gridextractor.h"
#include <memory>
#include "basictypes/debug.h"
#include "basictypes/minmaxbags.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

#ifdef XFEATURES2D
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#endif
namespace tslam{
float GridExtractor::getMinDescDistance()const{
    switch(_ftype){
    case F2D_GRID_ORB: return 50;
    case F2D_GRID_AKAZE: return 120;
    case F2D_GRID_BRISK: return 70;
    case F2D_GRID_FREAK:return 70;
    case F2D_GRID_SURF:return 0.125;
    default: return 0;
    };
}

  DescriptorTypes::Type GridExtractor::getDescriptorType()const{
      switch(_ftype){
      case F2D_ORB: return DescriptorTypes::DESC_ORB;
      case F2D_GRID_ORB: return DescriptorTypes::DESC_ORB;
      case F2D_GRID_AKAZE: return DescriptorTypes::DESC_AKAZE;
      case F2D_GRID_BRISK: return DescriptorTypes::DESC_BRISK;
      case F2D_GRID_FREAK:return DescriptorTypes::DESC_FREAK;
      case F2D_GRID_SURF:return DescriptorTypes::DESC_SURF;
      };
      throw std::runtime_error("Invalid type descriptor");
   }

  void GridExtractor::setParams(F2S_Type type )
  {   _ftype=type;
      _fextractors.clear();
  }


void GridExtractor::createDetectors(Feature2DSerializable::FeatParams  param,cv::Size imageSize){
std::cout<<"JJJ="<<str_params<<std::endl;
   _fextractors.clear();
    int gfeatures=param.getMaxFeatures()/param.nthreads;
     for(int i=0;i<param.nthreads;i++){
        switch(_ftype){
        case F2D_GRID_ORB:{
             auto orb=cv::ORB::create();
            orb->setMaxFeatures(gfeatures);
            orb->setScaleFactor(param.scaleFactor);
            orb->setEdgeThreshold(20);
            orb->setFastThreshold(7);
            orb->setNLevels(param.nOctaveLevels);
            _fextractors.push_back(orb);
            _overlapborder=20;
        }break;
        case F2D_GRID_AKAZE:{
            auto akaze=cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB,  0,  3, 1e-4f, param.nOctaveLevels);
             akaze->setThreshold(1e-4);
            _fextractors.push_back(akaze);
            _overlapborder=25;
        }break;
         case F2D_GRID_BRISK:{
            if (param.nOctaveLevels!=8)throw std::runtime_error("BRISK NEEDS 8 octave levels. No more no less");
            auto desc=cv::BRISK::create(30,5);
            _fextractors.push_back(desc);
            _overlapborder=25;
        }break;
        case F2D_GRID_FREAK:{
#ifdef XFEATURES2D
//            _nOctaveLayers=8;
            auto desc=cv::xfeatures2d::FREAK::create();
            _fextractors.push_back(desc);
            _fdetectors.push_back( cv::FastFeatureDetector::create(25));
            _overlapborder=25;
#else
            throw std::runtime_error("could not load xfeatures2d module");
#endif
        }break;
        case F2D_GRID_SURF:{
#ifdef XFEATURES2D
            _nOctaveLayers=4;
            auto desc=cv::xfeatures2d::SURF::create();
            _fextractors.push_back(desc);
            _overlapborder=25;
#else
            throw std::runtime_error("could not load xfeatures2d module");
#endif
            }break;
        default:
            throw std::runtime_error("could not create detector");
        };
    }

}
void GridExtractor::toStream_impl(std::ostream &str){
    str.write((char*)&_featParams,sizeof(_featParams));
    str.write((char*)&_ftype,sizeof(_ftype));
    str.write((char*)&_areParamsSet,sizeof(_areParamsSet));
    str.write((char*)&_overlapborder,sizeof(_overlapborder));
 }

void GridExtractor::fromStream_impl(std::istream &str){
    str.read((char*)&_featParams,sizeof(_featParams));
    str.read((char*)&_ftype,sizeof(_ftype));
    str.read((char*)&_areParamsSet,sizeof(_areParamsSet));
    str.read((char*)&_overlapborder,sizeof(_overlapborder));
}

void GridExtractor::detectAndCompute_thread(cv::InputArray _image, cv::InputArray _mask, cv::Ptr<cv::Feature2D> extractor, cv::Ptr<cv::Feature2D> detector, std::vector<cv::KeyPoint> &vkeypoints,  cv::Mat  &vdesc, int y){
  //  _debug_msg_ ("Thread:"<<y<<":"<<std::this_thread::get_id() );


    cv::Mat image=_image.getMat();
    cv::Mat mask=_mask.getMat();
    int ysize=image.rows/ _featParams.nthreads;
    //determine bounds. It must be slighly enlarged to detect elements in the border (only for patches in the central regions)
    int miny= std::max(0, (ysize*y)-_overlapborder);
    int maxy= std::min(image.rows, (y==_featParams.nthreads?image.rows:ysize*(y+1))+_overlapborder);
    //_debug_msg_ ("miny:"<<miny<<" "<<maxy );

    cv::Mat gridMask;
    if (!mask.empty()) gridMask=mask.rowRange(miny,maxy);

    vkeypoints.clear();

    if (!detector.empty()){

        std::vector<std::vector<cv::KeyPoint> > v_keypoints(_featParams.nOctaveLevels);
        std::vector<cv::Mat > v_desc(_featParams.nOctaveLevels);
        cv::Mat im=image.rowRange(miny,maxy),im2;
        float curScaleFactor=1;
        for(size_t level=0;level<v_desc.size();level++){
            detector->detect(im,v_keypoints[level]);
            extractor->compute(im,v_keypoints[level], v_desc[level]);
            for(auto &kp:v_keypoints[level]){
                kp.octave=level;
                kp.pt*=curScaleFactor;
            }
            cv::resize(im,im2,cv::Size(  float(im.cols)/1.2 ,float(im.rows)/1.2));
            im=im2;
            curScaleFactor*=1.2;

        }
        for(const auto & v_k:v_keypoints){
            for(const auto & kp:v_k)
                vkeypoints.push_back(kp);
        }

        vdesc.create(vkeypoints.size(),  v_desc[0].cols , v_desc[0].type());
        int curRow=0;
        for(const auto &desc:v_desc){
            auto roi=vdesc.rowRange(curRow,curRow+desc.rows);
            desc.copyTo(roi);
            curRow+=desc.rows;
        }

 //        detector->detect(image.rowRange(miny,maxy),vkeypoints,gridMask);
//        extractor->compute(image.rowRange(miny,maxy),vkeypoints, vdesc);
    }
    else{
        extractor->detectAndCompute(image.rowRange(miny,maxy),gridMask,vkeypoints, vdesc,false);
    }


//    cv::UMat imagex,maskx;
//    image.copyTo(imagex);
//    mask.copyTo(maskx);
  //  detector->detectAndCompute(imagex,maskx,*vkeypoints, *vdesc,false);

    int maxOct=0,minOct=1000;
    //add the offset to the keypoints
    for(auto &kp:vkeypoints) {
        kp.pt.y+=miny;
        maxOct=std::max(kp.octave,maxOct);
        minOct=std::min(kp.octave,minOct);
    }


}



void GridExtractor::detectAndCompute_impl(cv::InputArray _image, cv::InputArray _mask, std::vector<cv::KeyPoint> &Keypoints, cv::OutputArray Descriptors, FeatParams params){

    if (_fextractors.empty() || !(_featParams==params) ){
        createDetectors(params,_image.size());
        _featParams=params;
    }
    std::vector<std::vector<cv::KeyPoint>> vkeypoints(_featParams.nthreads);
    std::vector<cv::Mat> vdesc(_featParams.nthreads);
    //divide the work in threads

    std::vector<std::thread> vthreads;
    for(int i=0;i<_featParams.nthreads;i++){
        _debug_msg_("ssssy="<<i);
        cv::Ptr<cv::Feature2D> detector;
        if ( _fdetectors.size()!=0)  detector=_fdetectors[i];
        vthreads.push_back(std::thread(&GridExtractor::detectAndCompute_thread,this,_image,_mask, _fextractors[i], detector,std::ref( vkeypoints[i]), std::ref(vdesc[i]),i) );
    }
    //join to them
    for(int i=0;i<_featParams.nthreads;i++) vthreads[i].join();
    vthreads.clear();

    //count total number of features
    int tsize=0;
    for(auto &vkp:vkeypoints) tsize+=vkp.size();

//if less tham max, copy
    if (tsize<=_featParams.getMaxFeatures()){
        Keypoints.resize(tsize);
        Descriptors.create(tsize,vdesc[0].cols,vdesc[0].type());
        cv::Mat descriptors = Descriptors.getMat();
        assert(descriptors.isContinuous());
        int cur_kpidx=0;
        //size in chars of a descriptor
        int desc_charsize= vdesc[0].elemSize()*vdesc[0].cols/sizeof(char);

        for(size_t i=0;i<vkeypoints.size();i++){
            memcpy(&Keypoints[cur_kpidx],&vkeypoints[i][0],sizeof(cv::KeyPoint)*vkeypoints[i].size() );
            memcpy(descriptors.ptr<char>(cur_kpidx),vdesc[i].ptr<char>(0),desc_charsize*vkeypoints[i].size());
            cur_kpidx+=vkeypoints[i].size();
        }
    }
    else{//select only these with maximum response

        struct KeyPointInfo{
            float _response;
            int _idx,_thread;
            KeyPointInfo(const cv::KeyPoint & kp,int thread,int idx){
                _response=kp.response;
                _idx=idx;
                _thread=thread;
            }
            bool operator<(const KeyPointInfo &b)const{return  _response<b._response;           }
            bool operator>(const KeyPointInfo &b)const{return  _response>b._response;           }
        };
        MaxBag<KeyPointInfo> mbag;
        mbag.setMaxSize(_featParams.getMaxFeatures());
        for(size_t i=0;i<vkeypoints.size();i++ )
            for(size_t j=0;j<vkeypoints[i].size();j++ ){
                mbag.push(KeyPointInfo(vkeypoints[i][j],i,j));
            }
        //now, take the most interesting ones and add them to the output

        Keypoints.resize(_featParams.getMaxFeatures());
        Descriptors.create(_featParams.getMaxFeatures(),vdesc[0].cols,vdesc[0].type());
        cv::Mat descriptors = Descriptors.getMat();
        int cur_kpidx=0;
        int desc_charsize= vdesc[0].elemSize()*vdesc[0].cols/sizeof(char);
        while(!mbag.empty()){
            auto kpi=mbag.pop();
            Keypoints[cur_kpidx]= vkeypoints[kpi._thread][kpi._idx];
            memcpy(descriptors.ptr<char>(cur_kpidx),vdesc[kpi._thread].ptr<char>(kpi._idx),desc_charsize);
            cur_kpidx++;
        }
    }

    _debug_msg_("total size="<<Keypoints.size());
    //          cv::waitKey(0);
}


}
