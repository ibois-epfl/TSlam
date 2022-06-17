#ifndef VideoFrameIO_H

#define  VideoFrameIO_H

#include <iostream>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "featureextractors/feature2dserializable.h"
#include "utils/frameextractor.h"
#include "basictypes/timers.h"
#include "basictypes/cvversioning.h"
class VideoFrameWriter{
  public:
    cv::VideoCapture vcap;
    std::shared_ptr<reslam::Feature2DSerializable> f2ds;
    reslam::FrameExtractor Fextractor;
    std::ofstream outFile;
    reslam::ImageParams imageParams;
    cv::Size vsize;
    reslam::Params params;
    bool isHeaderWritten=false;

    void setParams(const reslam::Params &p, string video_path, string outFilePath,reslam::ImageParams &ip,float ImageScaleFactor, string descriptor,int nthreads,int maxFeatures,int nOctaves,float scaleFactor,string aruco_dict="ARUCO",float markerSize=1){
        imageParams=ip;


        params=p;
        params.maxFeatures=maxFeatures;
        params.nthreads_feature_detector=nthreads;
        params.nOctaveLevels=nOctaves;
        params.scaleFactor=scaleFactor;
        f2ds=reslam::Feature2DSerializable::create(reslam::DescriptorTypes::fromString(descriptor));
        params.aruco_markerSize=markerSize;
        //params.aruco_DetectorParams.dictionary=aruco_dict;
        Fextractor.setParams(f2ds,params);


        vcap.open(video_path);
        if (!vcap.isOpened())throw std::runtime_error("COud not open file:"+video_path);

        if (!vcap.grab()) throw std::runtime_error("COud not open file:"+video_path);
        cv::Mat image;
        vcap.retrieve(image);
        vsize=image.size();
        if (ImageScaleFactor!=1){
            vsize.width*=ImageScaleFactor;
            vsize.height*=ImageScaleFactor;
            cout<<"image size="<<vsize<<endl;
            imageParams.resize(vsize);
        }
        outFile.open(outFilePath,std::ios::binary);
        if (!outFile.is_open()) throw std::runtime_error("COud not open file:"+outFilePath);
        isHeaderWritten=false;

    }



    void setDetectMarkers(bool v){
        Fextractor.detectMarkers()=v;
    }
    reslam::Frame frame;
    int process(cv::Mat &im){
        if (!isHeaderWritten) writeHeader();
        vcap.retrieve(im);
        im=resize(im,vsize);
        Fextractor.process(im,imageParams,frame,vcap.get(CV_CAP_PROP_POS_FRAMES));
        frame.toStream(outFile);
        cout<<"sig="<<frame.fseq_idx<<" "<< frame.getSignature()<<endl;
        if (!vcap.grab()) return -1;
        else return vcap.get(CV_CAP_PROP_POS_FRAMES);
    }
    int getNFrames(){return vcap.get(CV_CAP_PROP_FRAME_COUNT);}



private:

    void writeHeader(){


        uint64_t sig=12321;
        outFile.write((char*)&sig,sizeof(sig));
        outFile.write((char*)&vsize,sizeof(vsize));
        imageParams.toStream(outFile);
        params.toStream(outFile);
        sig=12322;
      outFile.write((char*)&sig,sizeof(sig));
        isHeaderWritten=true;
    }
    cv::Mat resize(cv::Mat &in,cv::Size size){
        if (size.area()<=0)return in;
        cv::Mat ret;
        cv::resize(in,ret,size);  return ret;
    }
};

class VideoFrameReader{
  public:
    cv::VideoCapture vcap;
    std::shared_ptr<reslam::Feature2DSerializable> f2ds;
    reslam::FrameExtractor Fextractor;
    std::ifstream inFile;
    reslam::ImageParams imageParams;
    cv::Size vsize;
    reslam::Params params;

    void open( string video_path,string videoFramePath){
        vcap.open(video_path);
        if (!vcap.isOpened())throw std::runtime_error("COud not open file:"+video_path);

        inFile.open(videoFramePath,std::ios::binary);
        if (!inFile.is_open()) throw std::runtime_error("COud not open file:"+videoFramePath);
        uint64_t sig;
        inFile.read((char*)&sig,sizeof(sig));
        if (sig!=12321) throw std::runtime_error("file:"+videoFramePath+" is not of correct type");
        inFile.read((char*)&vsize,sizeof(vsize));
        imageParams.fromStream(inFile);
        params.fromStream(inFile);
        inFile.read((char*)&sig,sizeof(sig));
        if (sig!=12322) throw std::runtime_error("file:"+videoFramePath+" is not of correct type");
    }


    bool grab(cv::Mat &image,reslam::Frame &frame){
        try{
            if (!vcap.grab())return false;
            vcap.retrieve(image);
            image=resize(image,vsize);
            frame.fromStream(inFile);
            cout<<"sig="<<frame.fseq_idx<<" "<< frame.getSignature()<<endl;
            return true;
        }catch(std::exception &ex){return false;}
    }

    int getNFrames(){return vcap.get(CV_CAP_PROP_FRAME_COUNT);}
private:

    cv::Mat resize(cv::Mat &in,cv::Size size){
        if (size.area()<=0)return in;
        cv::Mat ret;
        cv::resize(in,ret,size);  return ret;
    }
};


#endif
