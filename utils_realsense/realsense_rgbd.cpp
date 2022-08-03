#include "tslamtypes.h"
#include "tslam.h"
#include "basictypes/debug.h"
#include "mapviewer.h"
#include "basictypes/timers.h"
#include "map.h"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "stdint.h"

class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

// Declare RealSense pipeline, encapsulating the actual device and sensors
std::shared_ptr<rs2::pipeline> pipe;
rs2::config cfg;
// Initialize a shared pointer to a device with the current device on the pipeline
rs2::device device;

bool getNextFrame(cv::Mat& in_depth, cv::Mat& in_image);
float get_depth_scale(rs2::device dev);

tslam::ImageParams getRealSenseImageParams(const std::shared_ptr<rs2::pipeline>& pipe){

    tslam::ImageParams  IP;

    rs2_intrinsics intr = pipe->get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();

    IP.CameraMatrix= (cv::Mat_<float>(3,3) <<  intr.fx,0,intr.ppx,0,intr.fy,intr.ppy,0,0,1);
    IP.Distorsion= (cv::Mat_<float>(1,5) <<  intr.coeffs[0],intr.coeffs[1], intr.coeffs[2],intr.coeffs[3],intr.coeffs[4]);
    IP.CamSize=cv::Size(intr.width,intr.height);
    IP.bl=0.095;//camera base line
    IP.rgb_depthscale=get_depth_scale(pipe->get_active_profile().get_device());//scale factor to convert into the desired scale

    return IP;
}
inline cv::Vec4f mult(cv::Vec4f point,const cv::Mat &T){
    const float *mat_ptr=T.ptr<float>(0);
    float x=mat_ptr[0]*point[0]+ mat_ptr[1]*point[1] + mat_ptr[2]*point[2]+mat_ptr[3];
    float y=mat_ptr[4]*point[0]+ mat_ptr[5]*point[1] + mat_ptr[6]*point[2]+mat_ptr[7];
    float z=mat_ptr[8]*point[0]+ mat_ptr[9]*point[1] + mat_ptr[10]*point[2]+mat_ptr[11];
    return cv::Vec4f(x,y,z,point[3]);
}
void savePcd(const cv::Mat &rgb,const cv::Mat &depth,const cv::Mat &C2G,string filepathpcd);


int main(int argc,char **argv){
    try {
        CmdLineParser cml(argc, argv);
        if (argc < 2 || cml["-h"]) {
            cerr << "Usage: (bagfile|live) [-in world]  [-out name] [-loc_only: use if only needs tracking (no SLAM)] [-d descriptor 0:orb 1:AKAZE] [-size markers_size] [-dict <dictionary>:. By default ARUCO_MIP_36h12]  [-nomarkers] [-vsize w:h] [-debug level] [-voc bow_volcabulary_file] [-fdt n:number of threads of the feature detector] [-st starts the processing stopped ] [-nokeypoints] [-marker_minsize <val_[0,1]>]  [-noX disabled windows] [-fps X: set video sequence frames per second] [-skip <n>:skips n frames before starting] [-outavi <filename>:saves to an avi file] [-KFMinConfidence <val>]"   << endl; return -1;
        }

        cv::VideoWriter vwrite;
        string videoPath=cml("-outavi","");

        pipe = std::make_shared<rs2::pipeline>();

        string bagFile;
        if (string(argv[1])!="live")
        {
            bagFile=argv[1];
            cfg.enable_device_from_file(bagFile,false);
            pipe->start(cfg); //File will be opened in read mode at this point
            device = pipe->get_active_profile().get_device();
            device.as<rs2::playback>().set_real_time(false);
        }
        else
        {
            rs2::context ctx;
            rs2::device_list devices = ctx.query_devices();
            if (devices.size() == 0)
                throw std::runtime_error("No device connected, please connect a RealSense device");

            cfg.enable_stream(RS2_STREAM_DEPTH);
            cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8);
            pipe->start(cfg);
            device = pipe->get_active_profile().get_device();
        }

        tslam::TSlam Slam;


        tslam::ImageParams image_params = getRealSenseImageParams(pipe);

        tslam::Params params;
        params.aruco_markerSize = stof(cml("-size", "1"));
        params.aruco_minMarkerSize= stod(cml("-marker_minsize", "0.025"));
        params.detectMarkers = !cml["-nomarkers"]; //work only with keypoints!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        params.detectKeyPoints = !cml["-nokeypoints"];
        params.runSequential = cml["-sequential"];
        params.nthreads_feature_detector = stoi(cml("-fdt", "2"));
        params.kpDescriptorType = tslam::DescriptorTypes::fromString(cml("-d", "orb"));
        if(cml["-KFMinConfidence"])
             params.KFMinConfidence=stof(cml("-KFMinConfidence"));

        std::shared_ptr<tslam::Map> TheMap=std::make_shared<tslam::Map>();
        if (cml["-in"])
            TheMap->readFromFile(cml("-in"));

        Slam.setParams( TheMap,params,cml("-voc"));

        if (cml["-loc_only"]) Slam.setMode(tslam::MODE_LOCALIZATION);

        bool showWindows=!cml["-noX"];

        int currentFrameIndex=0;

        char k=0;
        bool finish = false;
        tslam::MapViewer TheViewer;
        tslam::TimerAvrg Fps;
        cv::Mat in_image, in_depth;
        while(!finish && getNextFrame(in_depth, in_image)){

            Fps.start();
            cv::Mat pose_f2gT=Slam.processRGBD(in_image, in_depth, image_params,currentFrameIndex);
            Fps.stop();

            cout << "Image " << currentFrameIndex << " fps=" << 1./Fps.getAvrg()<< endl;

            if(showWindows)
                k=TheViewer.show( TheMap, in_image ,pose_f2gT,"#" + std::to_string(currentFrameIndex) + " fps=" + to_string(1./Fps.getAvrg()) );

            if (int(k) == 27)finish = true;//pressed ESC

            if (k=='e'){
                string number = std::to_string(currentFrameIndex);
                while (number.size() < 5) number = "0" + number;
//                Slam.saveToFile("world-"+number+ ".ucs");
                TheMap->saveToFile("world-"+number+ ".map");
            }
            if (k=='p'){
                cout<<"T="<<pose_f2gT<<endl;
                string number = std::to_string(currentFrameIndex);
                while (number.size() < 5) number = "0" + number;
                savePcd(in_image,in_depth,pose_f2gT,"depth-"+number+".pcd");
                cout<<"Saved:"<<"depth-"+number+".pcd"<<endl;
            }
            if (k=='r')
                Slam.clear();

            if (!videoPath.empty()){
                cv::Mat imwrite;
                TheViewer.getImage(imwrite);
                if (!vwrite.isOpened())
                    vwrite.open(videoPath, CV_FOURCC('X', '2', '6', '4'),stoi(cml("-fps","30")),imwrite.size()  , imwrite.channels()!=1);
                vwrite<<imwrite;
            }

            currentFrameIndex++;
        }

        TheMap->saveToFile(cml("-out","world") + ".map");
    }
    catch (std::exception &ex) {
        cerr << ex.what() << endl;
    }
}

bool getNextFrame(cv::Mat& in_depth, cv::Mat& in_image)
{
    rs2::align align_to_color(RS2_STREAM_COLOR);

    if (device.as<rs2::playback>())
    {
        rs2::playback playback = device.as<rs2::playback>();
        if(playback.current_status() == RS2_PLAYBACK_STATUS_STOPPED)
            return false;
    }

    rs2::frameset data = pipe->wait_for_frames();
    data = align_to_color.process(data);

    rs2::frame depth_frame;
    try{
        depth_frame = data.get_depth_frame();      // Find the depth data
    } catch (rs2::error& e) {
        throw std::runtime_error("Depth stream not found");
    }

    rs2::frame frame_color;
    try{
        frame_color = data.get_color_frame();      // Find the color data
    } catch (rs2::error& e) {
        throw std::runtime_error("Color stream not found");
    }

    const int w = depth_frame.as<rs2::video_frame>().get_width();
    const int h = depth_frame.as<rs2::video_frame>().get_height();
    in_depth = cv::Mat(cv::Size(w, h), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP).clone();

    const int wcolor = frame_color.as<rs2::video_frame>().get_width();
    const int hcolor = frame_color.as<rs2::video_frame>().get_height();
    in_image = cv::Mat(cv::Size(wcolor, hcolor), CV_8UC3, (void*)frame_color.get_data(), cv::Mat::AUTO_STEP).clone();

    return true;
}

float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

void savePcd(const cv::Mat &rgb,const cv::Mat &depth,const cv::Mat &C2G,string filepathpcd){

    auto getRGBfloat=[](cv::Vec3b color){
        float f;
        uchar *fc=(uchar*)&f;
        fc[0]=color[0];
        fc[1]=color[1];
        fc[2]=color[2];
        return f;
    };
    int nvalid=0;
    for(int y=0;y<depth.rows;y++){
        const uint16_t *_depth=depth.ptr<uint16_t>(y);
        for(int x=0;x<depth.cols;x++)
            if ( _depth[x]*1e-3<4 && _depth[x]!=0) nvalid++;
    }

    cv::Mat G2C=C2G.inv();
                ofstream file(filepathpcd);
file<<"# .PCD v.7 - Point Cloud Data file format"
<<endl<<"VERSION .7"
<<endl<<"FIELDS x y z rgb"
<<endl<<"SIZE 4 4 4 4"
<<endl<<"TYPE F F F F"
<<endl<<"COUNT 1 1 1 1"
<<endl<<"VIEWPOINT 0 0 0 1 0 0 0"
<<endl<<"WIDTH "<<nvalid
<<endl<<"HEIGHT "<<1
<<endl<<"POINTS "<<nvalid
<<endl<<"DATA binary"<<endl;
float fx_inv=1./517.306408;
float fy_inv=1./516.469215;
float cx=318.643040;
float cy=255.313989;
cv::Vec4f p;

for(int y=0;y<depth.rows;y++){
    const cv::Vec3b *_rgb=rgb.ptr<cv::Vec3b>(y);
    const uint16_t *_depth=depth.ptr<uint16_t>(y);
    for(int x=0;x<depth.cols;x++){
        if ( _depth[x]*1e-3<4 && _depth[x]!=0){
            //get the 3d
            p[2]= float(_depth[x])*1e-3;
            p[0]= p[2] * float( x - cx ) * fx_inv;
            p[1]= p[2] * float( y - cy ) * fy_inv;
            p[3]=getRGBfloat(_rgb[x]);
            p=mult(p,G2C);
           // cout<<_depth[x]<<" "<<p[0]<<" "<<p[1]<<" "<<p[2]<<endl;
            file.write((char*)&p,sizeof(float)*4);
        }
    }
}

}
