#include "slam.h"
#include "stuff/debug.h"
#include "mapviewer.h"
#include "stuff/timers.h"
#include "map.h"
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

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
    cv::GaussianBlur(ret,ret2,cv::Size(3,3),1);
    return ret2;
}


int cIndexLive=0;
int getCurrentFrameIndex(cv::VideoCapture &vcap,bool isLive){

    if (isLive)return cIndexLive++;
    else return  vcap.get(CV_CAP_PROP_POS_FRAMES);
}


void overwriteparamsbycommandLine(CmdLineParser &parser,reslam::Params &params){
    params.aruco_markerSize = stof(cml("-size", "1"));
    params.aruco_minSize = stod(cml("-marker_minsize", "0.025"));
    params.aruco_enclosedMarkers = cml["-em"];
    params.detectMarkers = !cml["-nomarkers"]; //work only with keypoints!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    params.detectKeyPoints = !cml["-nokeypoints"];
    params.runSequential = cml["-sequential"];
    params.maxFeatures = stoi(cml("-maxFeat","2000"));
    params.nOctaveLevels = stoi(cml("-nOct","8"));
    params.nthreads_feature_detector = stoi(cml("-fdt", "2"));
    params.fps=stof(cml("-fps","30"));
    params.kpDescriptorType = reslam::DescriptorTypes::fromString(cml("-desc", "orb"));

}

int main(int argc,char **argv){
	try {
		CmdLineParser cml(argc, argv);
        if (argc < 4 || cml["-h"]) {
            cerr << "Usage: (video|live[:cameraIndex(0,1...)])  camera_params.yml ucoslam_params.yml [-map world]  [-out name] "
                    "[-loc_only do not update map, only do localization. Requires -in] [-desc descriptor orb,akaze,brisk,freak] "
                    "[-msize markers_size] [-dict <dictionary>:. By default ARUCO_MIP_36h12]  "
                    "[-nomarkers] [-vf <float>:video resize factor] [-debug level] [-voc bow_volcabulary_file] "
                    "[-t_fe n:number of threads of the feature detector] [-st starts the processing stopped ] "
                    "[-nokeypoints] [-marker_minsize <val_[0,1]>] [-em . Uses enclosed markers] [-noX disabled windows] "
                    "[-fps X: set video sequence frames per second] [-outvideo filename]"
                    "[-maxFeat <int>:maximum number of features of the descriptor]"
                    "[-nOct <int>:number of octave layers]"
                 << endl; return -1;
		}
		reslam::debug::Debug::setLevel(stoi(cml("-debug", "0")));

		bool liveVideo = false;
		cv::VideoCapture vcap;
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
            vcap.set(CV_CAP_PROP_AUTOFOCUS, 0);
            liveVideo = true;

		}
		else vcap.open(argv[1]);

		if (!vcap.isOpened())
			throw std::runtime_error("Video not opened");

        reslam::Slam Slam;
		cv::Mat in_image;

        reslam::ImageParams image_params;
        image_params.readFromXMLFile(argv[2]);




        reslam::Params params;
        params.readFromYMLFile(argv[3]);

        auto TheMap=std::make_shared<reslam::Map>();
        if ( cml["-map"])
            TheMap->readFromFile(cml("-map"));

        if (cml["-in_debug"]) {
            Slam.readFromFile(cml("-in_debug"));
            params = Slam.getParams();
            //read until the last processed frame

            cerr << "skipping frames" << endl;
            vcap.set(CV_CAP_PROP_POS_FRAMES, Slam.getLastProcessedFrame());
            while (uint32_t(vcap.get(CV_CAP_PROP_POS_FRAMES)) <= Slam.getLastProcessedFrame()) {
                if (uint32_t(vcap.get(CV_CAP_PROP_POS_FRAMES)) % 10 == 0)cerr << uint32_t(vcap.get(CV_CAP_PROP_POS_FRAMES)) << " ";
                vcap >> in_image;
                cerr << endl;
            }
        }
        else {
            Slam.setParams(TheMap, params,cml("-voc"));
        }

        if (cml["-loc_only"]) Slam.setMode(reslam::Slam::MODE_LOCALIZATION);

        if (cml["-skip"]) {
            int n=stoi(cml("-skip","0"));
            cerr << "skipping frames" << endl;
            for(int i=0;i<n ;i++) {
                if (i%10==0)cerr<<i<<"/"<<n<<" ";
                vcap.grab();
            }
            cerr<<endl;
        }





        reslam::TimerAvrg Fps;

		while (in_image.empty())vcap >> in_image;

        cv::Size vsize(0,0);
        if (cml["-vf"]) {
            vsize.width=float(in_image.cols)*stof(cml("-vf"));
            vsize.height=float(in_image.rows)*stof(cml("-vf"));
            image_params.resize(vsize);
        }


		bool finish = false;
		auto TViewer = reslam::MapViewer::create(cml["-noX"] ? "" : "Cv");
		while (!finish && !in_image.empty()) {
			in_image = resize(in_image, vsize);
			int currentFrameIndex = getCurrentFrameIndex(vcap, liveVideo);
            if (reslam::debug::Debug::getLevel() >= 11 && currentFrameIndex % 10 == 0) {
				if (system("mkdir -p debug") == 0) {
					string number = std::to_string(currentFrameIndex);
					while (number.size() < 5) number = "0" + number;
					Slam.saveToFile("debug/debugworld-" + number + ".ucs");
				}
				Slam.saveToFile("debugworld-current.ucs");
			}
            Fps.start();
            Slam.process(in_image, image_params,currentFrameIndex);
            Fps.stop();
            cout << "Image " << currentFrameIndex << " fps=" << 1./Fps.getAvrg()<< endl;
            Slam.drawMatches(in_image);
           char k = TViewer->show(&Slam, in_image,"#" + std::to_string(currentFrameIndex) + " fps=" + to_string(1./Fps.getAvrg()) );
           if (int(k) == 27)finish = true;//pressed ESC

           if (!TheOutputVideo.empty()){
               auto image=TViewer->getImage();
               if(!videoout.isOpened())
                    videoout.open(TheOutputVideo, CV_FOURCC('X', '2', '6', '4'), reslam::Slam::getParams().fps,image.size()  , image.channels()!=1);
               if(videoout.isOpened())  videoout.write(image);
           }
           if (k=='r')
               Slam.clear();
           if (k=='e'){
               string number = std::to_string(currentFrameIndex);
               while (number.size() < 5) number = "0" + number;
               Slam.saveToFile("world-"+number+ ".ucs");
               TheMap->saveToFile("world-"+number+".map");
           }

			vcap >> in_image;
            if ( reslam::debug::Debug::getLevel() ==11)  {
                Slam.saveToFile("post_world.ucs");
            }
		}
        if(videoout.isOpened()) videoout.release();


        TheMap->saveToFile(cml("-out","world") +".map");
	}
	catch (std::exception &ex) {
		cerr << ex.what() << endl;
	}
}
