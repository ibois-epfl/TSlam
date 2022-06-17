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
#include "reslam.h"
#include "basictypes/debug.h"
#include "mapviewer.h"
#include "basictypes/timers.h"
#include "map.h"
#include "inputreader.h"
#include "basictypes/cvversioning.h"


cv::Mat getImage(cv::VideoCapture &vcap,int frameIdx){
    cv::Mat im;
    reslam::Frame frame;
    vcap.set(CV_CAP_PROP_POS_FRAMES,frameIdx);
    vcap.grab();
    vcap.set(CV_CAP_PROP_POS_FRAMES,frameIdx);
    vcap.retrieve(im);
    return im;
}


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

cv::Size readInpuSize(string s){
    for(auto &c:s)if(c==':')c =' ';
    stringstream sstr(s.c_str());
    cv::Size size;
    if ( sstr>>size.width>>size.height) return size;
    else return cv::Size(0,0);
}

cv::Mat resize(cv::Mat &in,cv::Size size){
    if (size.area()<=0)return in;
    cv::Mat ret,ret2;
    cv::resize(in,ret,size);  return ret;
}


int cIndexLive=0;
int getCurrentFrameIndex(cv::VideoCapture &vcap,bool isLive){

    if (isLive)return cIndexLive++;
    else return  int(vcap.get(CV_CAP_PROP_POS_FRAMES));
}


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

std::pair<std::string,std::string> getSplit(std::string str){
    std::string livestr=str;
    for(auto &c:livestr)if(c==':')c=' ';
    std::stringstream sstr;sstr<<livestr;
    std::string aux,aux2;int n=0;
    sstr>>aux>>aux2;
    return std::pair<std::string,std::string>(aux,aux2);
};
int main(int argc,char **argv){
	try {
		CmdLineParser cml(argc, argv);
        if (argc < 3 || cml["-h"]) {
            cerr << "Usage: (video|live[:cameraIndex(0,1...)])  camera_params.yml [-params ucoslam_params.yml] [-map world]  [-out name] [-scale <float>:video resize factor]"
                    "[-loc_only do not update map, only do localization. Requires -in]"
                    "\n"
                    "[-desc descriptor orb,akaze,brisk,freak] "
                    "[-aruco-markerSize markers_size] [-dict <dictionary>:. By default ARUCO_MIP_36h12]  "
                    "[-nomarkers]  [-debug level] [-voc bow_volcabulary_file] "
                    "[-t_fe n:number of threads of the feature detector] [-st starts the processing stopped ] "
                    "[-nokeypoints] [-marker_minsize <val_[0,1]>] [-em . Uses enclosed markers] [-nFoX disabled windows] "
                    "[-fps X: set video sequence frames per second] [-outvideo filename]"
                    "[-featDensity <float>:features density]"
                    "[-nOct <int>:number of octave layers]"
                    "[-noMapUpdate]"
                    "[-tfocus <float>: target focus employed to create the map. Replaces the one of the camera] "
                    "[-undistort] will undistort image before processing it"
                    "[-extra_params \"param1=value1 param2=value2...\"]"
                    "[-vspeed <int:def 1> video analysis speed ]"
                    "[-liveImageSize w:h]"

                 << endl; return -1;
		}

		bool liveVideo = false;
        InputReader vcap;
        cv::VideoWriter videoout;
		string TheInputVideo = string(argv[1]);
        string TheOutputVideo=cml("-outvideo");
		if (TheInputVideo.find("live") != std::string::npos)
		{
			int vIdx = 0;
			// check if the :idx is here
            char cad[100];
			if (TheInputVideo.find(":") != string::npos)
			{
				std::replace(TheInputVideo.begin(), TheInputVideo.end(), ':', ' ');
				sscanf(TheInputVideo.c_str(), "%s %d", cad, &vIdx);
			}
			cout << "Opening camera index " << vIdx << endl;
			vcap.open(vIdx);
            //vcap.set(CV_CAP_PROP_AUTOFOCUS, 0);
            liveVideo = true;

		}
        else vcap.open(argv[1],!cml["-sequential"]);

		if (!vcap.isOpened())
			throw std::runtime_error("Video not opened");

        reslam::ReSlam Slam;
		int debugLevel = stoi(cml("-debug", "0"));
        Slam.setDebugLevel(debugLevel);
        Slam.showTimers(true);
        reslam::ImageParams image_params;
        reslam::Params params;
        cv::Mat in_image;        

        image_params.readFromXMLFile(argv[2]);

        if( cml["-params"])        params.readFromYMLFile(cml("-params"));
        overwriteParamsByCommandLine(cml,params);

        auto TheMap=std::make_shared<reslam::Map>();
        //read the map from file?
        if ( cml["-map"]) TheMap->readFromFile(cml("-map"));

        //need to resize input image?
        cv::Size vsize(0,0);
//        cv::Size vsize(1200,880);
//        image_params.resize(vsize);


        if(!cml["-voc"]  && !cml["-map"])
        {
            cerr<<"Warning!! No VOCABULARY INDICATED. KEYPOINT RELOCALIZATION IMPOSSIBLE WITHOUT A VOCABULARY FILE!!!!!"<<endl;
        }


        if (cml["-loc_only"]) Slam.setMode(reslam::MODE_LOCALIZATION);

        //need to skip frames?
        if (cml["-skip"]) {
            int n=stoi(cml("-skip","0"));
            vcap.set(CV_CAP_PROP_POS_FRAMES,n);
            cerr<<endl;
        }
        if(cml["-liveImageSize"]){
            vcap.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );

            auto cameraSize=  getSplit( cml("-liveImageSize","-1:-1") );
            if(cameraSize.first!="-1")
                vcap.set(cv::CAP_PROP_FRAME_WIDTH,stoi(cameraSize.first));
            if(cameraSize.second!="-1")
                vcap.set(cv::CAP_PROP_FRAME_HEIGHT,stoi(cameraSize.second));
        }

        //read the first frame if not yet
        while (in_image.empty())
            vcap >> in_image;

//        std::cout << in_image.size() << std::endl;
//        image_params.resize(in_image.size());
        Slam.setParams(TheMap, params,cml("-voc"));

        //need undistortion

        bool undistort=cml["-undistort"];
        vector<cv::Mat > undistMap;
        if(undistort ){
            if( undistMap.size()==0){
                undistMap.resize(2);
                cv::initUndistortRectifyMap(image_params.CameraMatrix,image_params.Distorsion,cv::Mat(),cv::Mat(),image_params.CamSize,CV_32FC1,undistMap[0],undistMap[1]);
            }
            image_params.Distorsion.setTo(cv::Scalar::all(0));
        }
        //Create the viewer to see the images and the 3D
        reslam::MapViewer TheViewer;

        if (cml["-slam"]){
            Slam.readFromFile(cml("-slam"));
             vcap.set(CV_CAP_PROP_POS_FRAMES,Slam.getLastProcessedFrame());
            vcap.retrieve(in_image);
            vcap.set(CV_CAP_PROP_POS_FRAMES,Slam.getLastProcessedFrame());
            vcap.retrieve(in_image);
            TheMap=Slam.getMap();
            overwriteParamsByCommandLine(cml,params);
            Slam.updateParams(params);

        }

        if (cml["-noMapUpdate"])
            Slam.setMode(reslam::MODE_LOCALIZATION);


        int snapshot=stoi(cml("-snapshot","-1"));
        int processFrame=0;
        cv::Mat auxImage;
        //Ok, lets start
        reslam::TimerAvrg Fps;
        reslam::TimerAvrg FpsComplete;
        reslam::TimerAvrg TimerDraw;
        bool finish = false;
        cv::Mat camPose_c2g;
        int vspeed=stoi(cml("-vspeed","1"));
        while (!finish && !in_image.empty()) {
             FpsComplete.start();
            //image resize (if required)

             if (in_image.cols<in_image.rows ){
                 cv::Mat aux;
                 cv::transpose(in_image,aux);
                 cv::flip(aux,in_image,0);
             }
            in_image = resize(in_image, vsize);


            //image undistortion (if required)
            if(undistort ){               
                cv::remap(in_image,auxImage,undistMap[0],undistMap[1],cv::INTER_CUBIC);
                in_image=auxImage;
                image_params.Distorsion.setTo(cv::Scalar::all(0));
            }


            int currentFrameIndex = vcap.getNextFrameIndex()-1;
            Fps.start();
            camPose_c2g=Slam.process(in_image, image_params,currentFrameIndex);
            Fps.stop();
            if(cml["-show_signature"])
                cout<<"sig("<<currentFrameIndex<<")="<< Slam.getSignatureStr()<<endl;


            TimerDraw.start();
            //            Slam.drawMatches(in_image);
            //    char k = TheViewer.show(&Slam, in_image,"#" + std::to_string(currentFrameIndex) + " fps=" + to_string(1./Fps.getAvrg()) );
            char k =0;
            if(!cml["-noX"]) k=TheViewer.show(TheMap,   in_image, camPose_c2g,"#" + std::to_string(currentFrameIndex)/* + " fps=" + to_string(1./Fps.getAvrg())*/ ,Slam.getCurrentKeyFrameIndex());
            if (int(k) == 27 || k=='q')finish = true;//pressed ESC
            TimerDraw.stop();

             //save to output video?
            if (!TheOutputVideo.empty()){
                auto image=TheViewer.getImage();
                if(!videoout.isOpened())
                    videoout.open(TheOutputVideo, CV_FOURCC('X', '2', '6', '4'), stof(cml("-fps","30")),image.size()  , image.channels()!=1);
                if(videoout.isOpened())  videoout.write(image);
            }

            //reset?
            if (k=='r') Slam.clear();
            //write the current map
            if (k=='e'){
                string number = std::to_string(currentFrameIndex);
                while (number.size() < 5) number = "0" + number;
                TheMap->saveToFile("world-"+number+".map");
            }
            processFrame++;

            if (k=='v'){
                Slam.saveToFile("slam.slm");
            }


            if(cml["-snapshot"])
                if(processFrame%snapshot==0)
                    Slam.saveToFile(cml("-out","slam")+"_"+std::to_string(currentFrameIndex)+".slm");

            if(k=='s')
                TheMap->saveToFile(cml("-out","world") +".map");

            //read next
             vcap>>in_image;
            if(!camPose_c2g.empty()){
                for(int s=0;s<vspeed-1;s++)
                    vcap >> in_image;
            }
            FpsComplete.stop();
             cout << "Image " << currentFrameIndex << " fps=" << 1./Fps.getAvrg()<<" "<<1./FpsComplete.getAvrg()<< " draw="<<1./TimerDraw.getAvrg()<< (camPose_c2g.empty()?" not tracked":" tracked")<< endl;
        }

        //release the video output if required
        if(videoout.isOpened()) videoout.release();

        //save the output

        TheMap->saveToFile(cml("-out","world") +".map");
        //save also the parameters finally employed
        params.saveToYMLFile("ucoslam_params_"+cml("-out","world") +".yml");
        if (debugLevel >=10){
            Slam.saveToFile("slam.slm");
        }
        TheMap->saveToMarkerMap("markermap.yml");


    }
    catch (const std::exception &ex) {
        cerr << ex.what() << endl;
    }
}
