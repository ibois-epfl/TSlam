#include <iostream>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "videoframeio.h"
#include "featureextractors/feature2dserializable.h"
#include "utils/frameextractor.h"
#include "basictypes/timers.h"
using namespace std;

class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};


void drawKeyPoints(cv::Mat &image,reslam::Frame &f){
    int s=float(image.cols)/640.f;
    cv::Point2f ps(s,s);
    cv::Point2f ps2(s+3,s+3);
  //  cv::Mat org1=image.clone();

    for(auto kp:f.kpts){
        cv::rectangle(image,kp-ps,kp+ps,cv::Scalar(0,255,0),s);

    }

}

cv::Size readInpuSize(string s){
    for(auto &c:s)if(c==':')c =' ';
    stringstream sstr(s.c_str());
    cv::Size size;
    if ( sstr>>size.width>>size.height) return size;
    else return cv::Size(0,0);
}

int main(int argc,char **argv){
    CmdLineParser cml(argc,argv);
    try{

        if (argc<4 || cml["-h"]){
            cerr<<"USage: videoIn FrameSeqOut  ImageParamsFile [-params file] [-desc <descritor>:orb default] [-nOct <nOctaves> :8 default] [-s <scale>:1.2] "
                  "[-t_fe <nthreads>:2] [-maxFeat <max_features>:2000 default] [-nomarkers do not detect markers]"
                  "[-vf imagescaleFactor]"<<endl;
            return -1;
        }
        reslam::debug::Debug::setLevel(0);

        string descritor=cml("-desc","orb");
        int noctaves=stoi(cml("-nOct","8"));
        float sc=stof(cml("-s","1.2"));
        int nthreads=stoi(cml("-t_fe","2"));
        int maxFeatures=stoi(cml("-maxFeat","2000"));
        reslam::ImageParams ip;ip.readFromXMLFile(argv[3]);
        VideoFrameWriter vfp;
        reslam::Params params;
        if (cml["-params"]) params.readFromYMLFile(cml("-params"));
        vfp.setParams(params,argv[1],argv[2],ip, stof(cml("-vf","1")),descritor,nthreads,maxFeatures,noctaves,sc);

        int waitTime=10;
        if (cml["-nomarkers"]) vfp.setDetectMarkers(false);
        bool mustCOntinue=true;
        cv::Mat image;
        int nFrames=vfp.getNFrames();
        reslam::TimerAvrg Timer;
        while(mustCOntinue){
            Timer.start();
            int fp=vfp.process(image);
            Timer.stop();
            cerr<<fp<<"/"<<nFrames<<"/"<<Timer.getAvrg()*1000<<" ";
            if (fp==-1) mustCOntinue=false;
            float c=800.f/float(image.cols);
            cv::resize(image,image,cv::Size(800,float(image.rows)*c));
            cv::imshow("image",image);
            if ( cv::waitKey(10)==27)  mustCOntinue=false;
        }

        VideoFrameReader vfr;
        vfr.open(argv[1],argv[2]);
        reslam::Frame  f;
        mustCOntinue=true;
        while(vfr.grab(image,f) && mustCOntinue){
            drawKeyPoints(image,f);
            float c=800.f/float(image.cols);
            cv::resize(image,image,cv::Size(800,float(image.rows)*c));
            cv::imshow("image",image);
            char k=cv::waitKey(waitTime);
            if ( k==27)  mustCOntinue=false;
            if (k=='s') waitTime*=-1;
        }

    }   catch(std::exception &ex){
        std::cerr<<ex.what()<<std::endl;
    }

}
