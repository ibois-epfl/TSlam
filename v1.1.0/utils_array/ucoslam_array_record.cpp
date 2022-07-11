#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <aruco/marker.h>
#include <aruco/markerdetector.h>
#include "basictypes/cvversioning.h"

using namespace std;
double tavrg=0;
bool bPrintHelp=false;
bool showDetectedMarkers= false;
aruco::MarkerDetector MDetector;
bool recording=false;



void putText(cv::Mat &im,string text,cv::Point p,float size){
    float fact=float(im.cols)/float(640);
    if (fact<1) fact=1;

    cv::putText(im,text,p,cv::FONT_HERSHEY_SIMPLEX, size,cv::Scalar(0,0,0),3*fact);
    cv::putText(im,text,p,cv::FONT_HERSHEY_SIMPLEX, size,cv::Scalar(125,255,255),1*fact);
}

void printHelp(cv::Mat &im)
{
    float fs=float(im.cols)/float(1200);

    putText(im,"'m': show/hide detected markers",cv::Point(10,fs*60),fs*0.5f);
    putText(im,"'s': start/stop video capture",cv::Point(10,fs*80),fs*0.5f);
    putText(im,"'w': write image to file",cv::Point(10,fs*100),fs*0.5f);
}

void printInfo(cv::Mat &im){
    float fs=float(im.cols)/float(1200);
    putText(im,"fps="+to_string(1./tavrg),cv::Point(10,fs*20),fs*0.5f);
    putText(im,"'h': show/hide help",cv::Point(10,fs*40),fs*0.5f);
    if(bPrintHelp) printHelp(im);
}

class CmdLineParser{int argc; char **argv; public:
    CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}
    bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
    string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
};


int main(int argc,char **argv){
    try{
        CmdLineParser cml(argc, argv);

        if (argc<2){cerr<<"Usage: outputFolder dev1 dev2 ...  [-waitKey <delay> (-1 no window)][-d dictionary]"<<endl;return -1;}


        int params=2;
        int waitKey=10;
        if (cml["-waitKey"])
        {
            params+=2;
            waitKey=stoi(cml("-waitKey"));
        }

        string dictionary;
        if (cml["-dictionary"])
        {
            params+=2;
            dictionary=cml("-d");
        }

        int ncams=argc-params;
        std::cout << "Number of cameras: " <<ncams<<std::endl;

        string outputFolder = argv[1];

        vector<cv::VideoCapture> Cameras(ncams);
        int index=2;
        for(auto &Cam:Cameras){
            Cam.set(CV_CAP_PROP_AUTOFOCUS,0);

            cout<<"Connecting to "<<argv[index]<<endl;
            Cam.open(argv[index]);

            if( !Cam.isOpened() )
            {
                string cameraid = argv[index];
                cerr << "Error opening camera "+cameraid <<std::endl;
            }
            index++;
        }

        char key=0;
        double tinit=cv::getTickCount();
        int imgIdx=0;
        int saved=0;

        if (cml["-d"])
            MDetector.setDictionary(cml("-d"), 0.f);

        //Output videos
        vector<cv::VideoWriter> outfile(ncams);

        //Output images
        vector<cv::Mat> images(ncams);
        cv::Size outCamSize = cv::Size(640,480);

         while(key!=27){

             int index=0;
             for(int c=0; c<ncams; c++)
             {
                 cv::Mat frame;
                 Cameras[c] >> images[c];
             }

             if (recording)
                 for(int c=0; c<ncams; c++)
                    outfile[c].write(images[c]);

             cv::Mat matDst(cv::Size(outCamSize.width*ncams,outCamSize.height),CV_8UC3,cv::Scalar(255,255,0));
             std::string cam_name= argv[index+1];
             if (waitKey!=-1)
             {
                 for(int c=0; c<ncams; c++)
                 {
                     cv::Mat dst;
                     cv::resize(images[c],dst,cv::Size(640,480));
                     dst.convertTo(dst, CV_8UC3);

                     std::vector<aruco::Marker> TheMarkers;
                     if(showDetectedMarkers)
                         TheMarkers = MDetector.detect(dst);
                     for (unsigned int i = 0; i < TheMarkers.size(); i++)
                         TheMarkers[i].draw(dst, cv::Scalar(0, 0, 255),2,true);

                     if(recording)
                        putText(dst, "Recording cam"+to_string(c),cv::Point(200, 450), 1);
                     else
                        putText(dst, "cam"+to_string(c),cv::Point(300, 450), 1);


                     dst.copyTo(matDst(cv::Rect(images[c].cols*c,0,images[c].cols,images[c].rows)));
                 }
             }

             printInfo(matDst);
             imshow("result",matDst);

             key=cv::waitKey(waitKey);
             if (key=='w')
             {
                 int camIdx=0;
                 for(auto im:images)
                 {
                     string img_name= outputFolder + "/cam"+std::to_string(camIdx++)+"_"+std::to_string(saved)+".jpg";
                     cv::imwrite(img_name,im);
                     cout<<"saved "<<img_name<<endl;
                 }
                 saved++;
             }

             if (key=='s')
             {                 
                 recording=!recording;
                 if(recording)
                 {
                     for(int c=0;c<ncams;c++)
                     {
                         if(!outfile[c].isOpened())
                            if(!outfile[c].open(outputFolder + "/cam"+std::to_string(c)+".avi", CV_FOURCC('M', 'J', 'P', 'G'), 15,
                                         cv::Size(Cameras[c].get(CV_CAP_PROP_FRAME_WIDTH),Cameras[c].get(CV_CAP_PROP_FRAME_HEIGHT))))
                                    throw std::runtime_error("Could not open output video cam"+std::to_string(c)+".avi");
                     }
                     std::cout << "Start recording"<<std::endl;
                 }
                 else
                     std::cout << "Stop recording"<<std::endl;
             }

             if (key=='h')
                 bPrintHelp=!bPrintHelp;

             if (key=='m')
             {
                 showDetectedMarkers=!showDetectedMarkers;
                 if(showDetectedMarkers)
                    std::cout << "Marker detector enabled"<<std::endl;
                 else
                    std::cout << "Marker detector disabled"<<std::endl;
             }

            imgIdx++;
            tavrg=(cv::getTickCount()-tinit)/double(imgIdx*cv::getTickFrequency());
        }

        for(int c=0; c<ncams; c++)
        {
            std::cout << "Closes cam"+std::to_string(c)<< std::endl;
            Cameras[c].release();
            outfile[c].release();
        }
    }catch(std::exception &ex)
    {
        cout<<ex.what()<<endl;
    }
}
