#include "reslam.h"
#include "map.h"
#include "mapviewer.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "basictypes/debug.h"
#include "stereorectify.h"
#include "basictypes/cvversioning.h"
#include "imageparams.h"
#include <xflann/xflann.h>
#include <opencv2/features2d/features2d.hpp>
#include "basictypes/timers.h"


class CmdLineParser{int argc; char **argv;
                public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                    std::vector<std::string> getAllInstances(string str){
                        std::vector<std::string> ret;
                        for(int i=0;i<argc-1;i++){
                            if (string(argv[i])==str)
                                ret.push_back(argv[i+1]);
                        }
                        return ret;
                    }
                   };


void overwriteParamsByCommandLine(CmdLineParser &cml,reslam::Params &params){
    if ( cml["-aruco-markerSize"])      params.aruco_markerSize = stof(cml("-aruco-markerSize", "1"));
    if ( cml["-marker_minsize"])    params.aruco_minMarkerSize= stod(cml("-marker_minsize", "0.025"));
    if (cml["-nokeypoints"])params.detectKeyPoints=false;
    if (cml["-nomarkers"])  params.detectMarkers =false;
    if (cml["-sequential"]) params.runSequential=true;
    if (cml["-maxFeatures"])    params.maxFeatures = stoi(cml("-maxFeatures","4000"));
    if (cml["-nOct"])       params.nOctaveLevels = stoi(cml("-nOct","8"));
    if (cml["-fdt"])        params.nthreads_feature_detector = stoi(cml("-fdt", "2"));
     if (cml["-desc"])       params.kpDescriptorType = reslam::DescriptorTypes::fromString(cml("-desc", "orb"));
    if (cml["-dict"])       params.aruco_Dictionary = cml("-dict");
    if (cml["-tfocus"])  params.targetFocus =stof(cml("-tfocus","-1"));
    if (cml["-KFMinConfidence"])  params.KFMinConfidence =stof(cml("-KFMinConfidence"));
    if(cml["-nonmax"])    params.KPNonMaximaSuppresion=true;
    if(cml["-saveImages"])    params.saveImageInMap=true;

    if(cml["-autoAdjustKpSensitivity"])    params.autoAdjustKpSensitivity=true;
    if(cml["-extra_params"])    params.extraParams=cml("-extra_params");

    if(cml["-scale"]) params.kptImageScaleFactor=stof(cml("-scale"));

    if(cml["-nokploopclosure"]) params.reLocalizationWithKeyPoints=false;
    if(cml["-inplanemarkers"]) params.inPlaneMarkers=true;
    params.aruco_CornerRefimentMethod=cml("-aruco-cornerRefinementM","CORNER_SUBPIX");

    if (cml["-dbg_str"])
        reslam::debug::Debug::addString(cml("-dbg_str"),"");

    if(cml["-minFocalLength"])params.minFocalLength=stof(cml("-minFocalLength","200"));
    if(cml["-featuresFirstLevel"])params.featuresFirstLevel=stoi(cml("-featuresFirstLevel","140"));
    if(cml["-featuresFactor"])params.featuresFactor=stof(cml("-featuresFactor","1"));
    if(cml["-nKFMatcher"]) params.nKFMatcher=stof(cml("-nKFMatcher","0.3"));
}

int main(int argc, char* argv[]){

    CmdLineParser cml(argc,argv);
    if(argc<4 || cml["-h"]){
        cout<<"Usage: <stereo_calibration_file> <video1> <video2> [-voc path] [-params ucoslamparams.yml]"<<endl;
        return -1;
    }

    reslam::ImageParams imageParams;
    imageParams.readArrayFromXMLFile(argv[1]);

    std::vector<cv::VideoCapture> video;
    video.resize(imageParams.arraySize());
    for(size_t i=0;i<video.size();i++)
    {
        std::cout <<"Opening: "<< argv[i+2] << std::endl;
        video[i].open(argv[i+2]);
        if(!video[i].isOpened())
            throw runtime_error(string("Cannot open video file atx:")+argv[i+2]);
    }

    reslam::Params sparams;
    sparams.runSequential=cml["-sequential"];

    if(cml["-params"])
        sparams.readFromYMLFile(cml("-params"));
    overwriteParamsByCommandLine(cml,sparams);

    auto smap=make_shared<reslam::Map>();
    //read the map from file?
    if ( cml["-map"]) smap->readFromFile(cml("-map"));

    reslam::ReSlam system;
    reslam::MapViewer mv;
    system.setParams(smap,sparams,cml("-voc",""));



    std::vector<cv::Mat> images;
    images.resize(2);

    if (cml["-slam"]){
        system.readFromFile(cml("-slam"));
        for(size_t i=0;i<video.size();i++)
        {
            float nb_frames = (float)video[i].get(cv::CAP_PROP_FRAME_COUNT);

            std::cout <<"NB"<< nb_frames<<std::endl;
            video[i].set(CV_CAP_PROP_POS_FRAMES,system.getLastProcessedFrame());
            video[i].retrieve(images[i]);
            video[i].set(CV_CAP_PROP_POS_FRAMES,system.getLastProcessedFrame());
            video[i].retrieve(images[i]);
        }
        smap=system.getMap();
        overwriteParamsByCommandLine(cml,sparams);
        system.updateParams(sparams);
    }

    reslam::TimerAvrg Fps;
    reslam::TimerAvrg TimerDraw;
    reslam::TimerAvrg FpsComplete;

    if (cml["-loc_only"]) system.setMode(reslam::MODE_LOCALIZATION);


    int vspeed=stoi(cml("-vspeed","1"));
    char key=0;
    int snapshot=stoi(cml("-snapshot","-1"));
    int processFrame=0;

    while( video[0].grab() && video[1].grab() && key!=27){
        FpsComplete.start();
        int frameNumber=video[0].get(cv::CAP_PROP_POS_FRAMES);

        for(size_t i=0;i<video.size();i++)
            video[i].retrieve(images[i]);

        Fps.start();
        cv::Mat pose=system.processArray(images, imageParams, frameNumber);
        Fps.stop();
        TimerDraw.start();
        if(!cml["-noX"]) key=mv.show(smap,images[0],pose,"");
        TimerDraw.stop();

        FpsComplete.stop();
        processFrame++;

        if (key=='v'){
            system.saveToFile(cml("-out","slam")+"_"+std::to_string(frameNumber)+".slm");
        }

        if(cml["-snapshot"])
            if(processFrame%snapshot==0)
                system.saveToFile(cml("-out","slam")+"_"+std::to_string(frameNumber)+".slm");

        cout << "Image " << frameNumber << " fps=" << 1./Fps.getAvrg()<<" "<<1./FpsComplete.getAvrg()<< " draw="<<1./TimerDraw.getAvrg()<< (pose.empty()?" not tracked":" tracked")<< endl;
        cout << "sig="<<system.getSignatureStr()<<std::endl;
        //read next

        for(int s=0;s<vspeed-1;s++)
            for(size_t i=0;i<video.size();i++)
                video[i].grab();

    }
    smap->saveToFile(cml("-out","world") +".map");
    //sparams.saveToYMLFile(cml("-out","world") +".yml");

    return 0;
}
