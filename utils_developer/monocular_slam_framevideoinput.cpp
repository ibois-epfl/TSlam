#include "system.h"
#include "stuff/debug.h"
#include "mapviewer.h"
#include "stuff/timers.h"
#include "map.h"
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};



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


void overwriteParamsByCommandLine(CmdLineParser &cml,reslam::Params &params){
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
        if (argc < 3 || cml["-h"]) {
            cerr << "Usage:  invideo inframesequence [-map world]  [-out name] "
                    "[-loc_only do not update map, only do localization. "
                    "[-debug level] [-voc bow_volcabulary_file] "
                    "[-st starts the processing stopped ]  [-noX disabled windows] "
                    "[-outvideo filename]"
                 << endl; return -1;
        }
        reslam::debug::Debug::setLevel(stoi(cml("-debug", "0")));

        VideoFrameReader vcap;
        cv::VideoWriter videoout;
        string TheInputVideo = string(argv[1]);
        vcap.open(argv[1],argv[2]);

        reslam::Slam Slam;
        cv::Mat in_image;
        reslam::Frame in_frame;

        reslam::Params params=vcap.params;
        overwriteParamsByCommandLine(cml,params);

        auto TheMap=std::make_shared<reslam::Map>();
        if ( cml["-map"]) TheMap->readFromFile(cml("-map"));

//        if (cml["-in_debug"]) {
//            Slam.readFromFile(cml("-in_debug"));
//            params = Slam.getParams();
//            //read until the last processed frame

//            cerr << "skipping frames" << endl;
//            vcap.set(CV_CAP_PROP_POS_FRAMES, Slam.getLastProcessedFrame());
//            while (uint32_t(vcap.get(CV_CAP_PROP_POS_FRAMES)) <= Slam.getLastProcessedFrame()) {
//                if (uint32_t(vcap.get(CV_CAP_PROP_POS_FRAMES)) % 10 == 0)cerr << uint32_t(vcap.get(CV_CAP_PROP_POS_FRAMES)) << " ";
//                vcap >> in_image;
//                cerr << endl;
//            }
//        }
//        else {
            Slam.setParams(TheMap, params,cml("-voc"));
//        }

        if (cml["-loc_only"]) Slam.setMode(reslam::Slam::MODE_LOCALIZATION);

        if (cml["-skip"]) {
            int n=stoi(cml("-skip","0"));
            cerr << "skipping frames" << endl;
            for(int i=0;i<n ;i++) {
                if (i%10==0)cerr<<i<<"/"<<n<<" ";
                vcap.grab(in_image,in_frame);
            }
            cerr<<endl;
        }

        reslam::TimerAvrg Fps;
        bool finish = false;
        auto TViewer = reslam::MapViewer::create(cml["-noX"] ? "" : "Cv");
        while (!finish && vcap.grab(in_image,in_frame)) {

            Fps.start();
            Slam.process(in_frame);
            Fps.stop();
            cout << "Image " << in_frame.fseq_idx << " fps=" << 1./Fps.getAvrg()<< endl;
            Slam.drawMatches(in_image);
           char k = TViewer->show(&Slam, in_image,"#" + std::to_string( in_frame.fseq_idx) + " fps=" + to_string(1./Fps.getAvrg()) );
           if (int(k) == 27)finish = true;//pressed ESC

           //write video
           if (cml["-outvideo"]){
               auto image=TViewer->getImage();
               if(!videoout.isOpened())
                    videoout.open(cml("-outvideo"), CV_FOURCC('X', '2', '6', '4'), reslam::Slam::getParams().fps,image.size()  , image.channels()!=1);
               if(videoout.isOpened())  videoout.write(image);
           }
           if (k=='r') Slam.clear();
           if (k=='e'){
               string number = std::to_string( in_frame.fseq_idx);
               while (number.size() < 5) number = "0" + number;
               Slam.saveToFile("world-"+number+ ".ucs");
               TheMap->saveToFile("world-"+number+".map");
           }
           std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if(videoout.isOpened()) videoout.release();
        TheMap->saveToFile(cml("-out","world")  +".map");
    }
    catch (std::exception &ex) {
        cerr << ex.what() << endl;
    }
}

