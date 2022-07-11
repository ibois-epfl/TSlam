#include "ucoslam.h"
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


int main(int argc, char* argv[]){

    CmdLineParser cml(argc,argv);
    if(argc<4 || cml["-h"]){
        cout<<"Usage: <stereo_calibration_file> <video1> <video2> [-voc path] [-params ucoslamparams.yml]"<<endl;
        return -1;
    }

    ucoslam::ImageParams imageParams;
    imageParams.readArrayFromXMLFile(argv[1]);

    std::vector<cv::VideoCapture> video;
    video.resize(imageParams.arraySize());
    for(size_t i=0;i<video.size();i++)
    {
        video[i].open(argv[i+2]);
        if(!video[i].isOpened())
            throw runtime_error(string("Cannot open video file at:")+argv[i+1]);
    }

    ucoslam::Params sparams;
    sparams.detectMarkers=false;
    sparams.KFMinConfidence=0.8;
    sparams.runSequential=cml["-sequential"];

    if(cml["-params"])
        sparams.readFromYMLFile(cml("-params"));

    auto smap=make_shared<ucoslam::Map>();
    ucoslam::UcoSlam system;
    ucoslam::MapViewer mv;
    system.setParams(smap,sparams,cml("-voc",""));

    std::vector<cv::Mat> images;
    images.resize(2);

    char key=0;
    while( video[0].grab() && video[1].grab() && key!=27){
        int frameNumber=video[0].get(CV_CAP_PROP_POS_FRAMES);

        for(size_t i=0;i<video.size();i++)
            video[i].retrieve(images[i]);

        cv::Mat pose=system.processArray(images, imageParams, frameNumber);
        key=mv.show(smap,images[0],pose,"");
    }
    smap->saveToFile("world.map");

    return 0;
}
