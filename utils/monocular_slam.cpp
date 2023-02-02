/**
* This file is part of  TSLAM
*
* Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)
*
* TSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* TSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with UCOSlam-> If not, see <http://wwmap->gnu.org/licenses/>.
*/

#include "tslam.h"
#include "basictypes/debug.h"
#include "mapviewer.h"
#include "basictypes/timers.h"
#include "inputreader.h"
#include "basictypes/cvversioning.h"


// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;

}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat R)
{

//    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);

}

cv::Vec4f rotationMatrixToQuaternion(cv::Mat R)
{
    cv::Vec4f q;

    q[0] = sqrt(1.0 + R.at<float>(0,0) + R.at<float>(1,1) + R.at<float>(2,2)) / 2.0;
    q[1] = (R.at<float>(2,1) - R.at<float>(1,2)) / (4.0 * q[0]);
    q[2] = (R.at<float>(0,2) - R.at<float>(2,0)) / (4.0 * q[0]);
    q[3] = (R.at<float>(1,0) - R.at<float>(0,1)) / (4.0 * q[0]);

    return q;
}

cv::Mat getImage(cv::VideoCapture &vcap,int frameIdx){
    cv::Mat im;
    tslam::Frame frame;
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


void overwriteParamsByCommandLine(CmdLineParser &cml,tslam::Params &params){
    if (cml["-aruco-markerSize"])      params.aruco_markerSize = stof(cml("-aruco-markerSize", "1"));
    if (cml["-marker_minsize"])    params.aruco_minMarkerSize= stod(cml("-marker_minsize", "0.025"));
    if (cml["-nokeypoints"])        params.detectKeyPoints=false;
    if (cml["-nomarkers"])  params.detectMarkers =false;
    if (cml["-sequential"]) params.runSequential=true;
    if (cml["-maxFeatures"])    params.maxFeatures = stoi(cml("-maxFeatures","4000"));
    if (cml["-nOct"])       params.nOctaveLevels = stoi(cml("-nOct","8"));
    if (cml["-fdt"])        params.nthreads_feature_detector = stoi(cml("-fdt", "1"));
    if (cml["-desc"])       params.kpDescriptorType = tslam::DescriptorTypes::fromString(cml("-desc", "orb"));
    if (cml["-dict"])       params.aruco_Dictionary = cml("-dict");
    if (cml["-tfocus"])  params.targetFocus =stof(cml("-tfocus","-1"));
    if (cml["-KFMinConfidence"])  params.KFMinConfidence =stof(cml("-KFMinConfidence"));
    if(cml["-nonmax"])    params.KPNonMaximaSuppresion=true;
    if(cml["-saveImages"])    params.saveImageInMap=true;
    if(cml["-isInstancing"]) params.isInstancing=true;
    if(cml["-autoAdjustKpSensitivity"])    params.autoAdjustKpSensitivity=true;
    if(cml["-extra_params"])    params.extraParams=cml("-extra_params");

    if(cml["-scale"]) params.kptImageScaleFactor=stof(cml("-scale"));
    if(cml["-nokploopclosure"]) params.reLocalizationWithKeyPoints=false;
    if(cml["-inplanemarkers"]) params.inPlaneMarkers=true;
    params.aruco_CornerRefimentMethod=cml("-aruco-cornerRefinementM","CORNER_SUBPIX");

    if(cml["-enableLoopClosure"]) params.enableLoopClosure=true;
    else params.enableLoopClosure=false;

    if (cml["-dbg_str"])
        tslam::debug::Debug::addString(cml("-dbg_str"),"");
}

std::pair<std::string,std::string> getSplit(std::string str){
    std::string livestr=str;
    for(auto &c:livestr)if(c==':')c=' ';
    std::stringstream sstr;sstr<<livestr;
    std::string aux,aux2;int n=0;
    sstr>>aux>>aux2;
    return std::pair<std::string,std::string>(aux,aux2);
};

void enhanceImageBGR(const cv::Mat &imgIn, cv::Mat &imgOut){
    // Start CLAHE Contrast Limited and Adaptive Histogram Equalization

    //Get Intesity imagef
    cv::Mat Lab_image;
    cvtColor(imgIn, Lab_image, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> Lab_planes(3);
    cv::split(Lab_image, Lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    // clahe->setTilesGridSize(cv::Size(10, 10));
    cv::Mat clahe_L;
    clahe->apply(Lab_planes[0], clahe_L);

    // Merge the color planes back into an Lab image
    clahe_L.copyTo(Lab_planes[0]);
    cv::merge(Lab_planes, Lab_image);

    // convert back to RGB
    cv::cvtColor(Lab_image, imgOut, cv::COLOR_Lab2BGR);
}

void drawMesh(cv::Mat img, vector<cv::Vec3f> linePoints, cv::Mat projectionMatrix) {
    for(auto &p:linePoints){
        cv::Mat p4 = (cv::Mat_<float>(4, 1) << p[0], p[1], p[2], 1);
        cv::Mat p2 = projectionMatrix * p4;
        p2 /= p2.at<float>(2, 0);

        cout << projectionMatrix << endl;
        cout << p2 << endl;


        int x = img.cols - (p2.at<float>(0, 0) + 1.0f) / 2.0f * img.cols;
        int y = img.rows - (p2.at<float>(1, 0) + 1.0f) / 2.0f * img.rows;

        cout << x << "  " << y << endl;
        cv::circle(img, cv::Point(x, y), 5, cv::Scalar(200, 0, 200), 5);
    }
}

vector<cv::Vec3f> linePoints = {{-1.3046491146087646e+00,-5.1872926950454712e-01, 1.5634726524353027e+01}};
cv::Mat projectionMatrix;
int currentFrameIndex;

int main(int argc,char **argv){
    std::ios_base::sync_with_stdio(false);

    bool errorFlag = false;
    CmdLineParser cml(argc, argv);
    if (argc < 3 || cml["-h"]) {
        cerr << "Usage: (video|live[:cameraIndex(0,1...)])  camera_params.yml [-params tslam_params.yml] [-map world]  [-out name] [-scale <float>:video resize factor]"
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
                "[-outCamPose <filename>]"
                << endl;        
        return -1;
    }

    bool liveVideo = false;
    InputReader vcap;
    cv::VideoWriter videoout;
    string TheInputVideo = string(argv[1]);
    string TheOutputVideo = cml("-outvideo");
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

    tslam::TSlam *Slam = new tslam::TSlam;
    int debugLevel = stoi(cml("-debug", "0"));
    Slam->setDebugLevel(debugLevel);
    Slam->showTimers(true);
    tslam::ImageParams image_params;
    tslam::Params params;
    cv::Mat in_image;

    // load camera matrix and distortion coefficients
    image_params.readFromXMLFile(argv[2]);
    // update projection matrix for rendering mesh
    cv::Mat cameraMatrix = image_params.CameraMatrix;
    float camW = image_params.CamSize.width;
    float camH = image_params.CamSize.height;
    float x0 = 0, y0 = 0,zF = 100.0f, zN =0.01f;
    float fovX = cameraMatrix.at<float>(0,0);
    float fovY = cameraMatrix.at<float>(1,1);
    float cX = cameraMatrix.at<float>(0,2);
    float cY = cameraMatrix.at<float>(1,2);
    float perspectiveProjMatrixData[16] = {
            2 * fovX / camW,    0, ( camW - 2 * cX + 2 * x0) / camW,                         0,
            0,    2 * fovY / camH, (-camH + 2 * cY + 2 * y0) / camH,                         0,
            0,                  0,             (-zF - zN)/(zF - zN),  -2 * zF * zN / (zF - zN),
            0,                  0,                               -1,                         0
    };
    cv::Mat perspectiveProjMatrix(4, 4, CV_32F, perspectiveProjMatrixData);
    projectionMatrix = perspectiveProjMatrix.t();

    if( cml["-params"]) params.readFromYMLFile(cml("-params"));
    overwriteParamsByCommandLine(cml,params);

    params.enableLoopClosure = false;

    auto TheMap = std::make_shared<tslam::Map>();
    //read the map from file?
    if (cml["-map"]){
        TheMap->readFromFile(cml("-map"));
        cout << endl;
    }

    Slam->setParams(TheMap, params, cml("-voc"));

    if(!cml["-voc"]  && !cml["-map"])
    {
        cerr<<"Warning!! No VOCABULARY INDICATED. KEYPOINT RELOCALIZATION IMPOSSIBLE WITHOUT A VOCABULARY FILE!!!!!"<<endl;
    }

    ofstream outCamPose;
    bool toSaveCamPose = cml["-outCamPose"];
    if(toSaveCamPose) {
        outCamPose.open(cml("-outCamPose"));
        // outPose<<"frame_number pos_x pos_y pos_z rot_w rot_x rot_y rot_z"<<endl;
    }

    // if (cml["-loc_only"]) Slam->setMode(tslam::MODE_LOCALIZATION);

    // need to skip frames?
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
    //need to resize input image?
    cv::Size vsize(0,0);
    //need undistortion

    bool undistort=cml["-undistort"];
    vector<cv::Mat > undistMap;
    if(undistort ){
        cout << "undistort: on" << endl;
        if( undistMap.size()==0){
            undistMap.resize(2);
            cv::initUndistortRectifyMap(image_params.CameraMatrix,image_params.Distorsion,cv::Mat(),cv::Mat(),image_params.CamSize,CV_32FC1,undistMap[0],undistMap[1]);
        }
        image_params.Distorsion.setTo(cv::Scalar::all(0));
    }
    //Create the viewer to see the images and the 3D
    tslam::MapViewer TheViewer;

//    if (cml["-slam"]){
//        Slam->readFromFile(cml("-slam"));
//            vcap.set(CV_CAP_PROP_POS_FRAMES,Slam->getLastProcessedFrame());
//        vcap.retrieve(in_image);
//        vcap.set(CV_CAP_PROP_POS_FRAMES,Slam->getLastProcessedFrame());
//        vcap.retrieve(in_image);
//        TheMap=Slam->getMap();
//        overwriteParamsByCommandLine(cml,params);
//        Slam->updateParams(params);

//    }

//    if (cml["-noMapUpdate"])
//        Slam->setMode(tslam::MODE_LOCALIZATION);



    cv::Mat auxImage;
    //Ok, lets start
    tslam::TimerAvrg Fps;
    tslam::TimerAvrg FpsComplete;
    tslam::TimerAvrg TimerDraw;
    bool finish = false;
    cv::Mat camPose_c2g;
    int vspeed=stoi(cml("-vspeed","1"));
    while (!finish && !in_image.empty())  {
        try{
            FpsComplete.start();

            //image resize (if required)
            in_image = resize(in_image, vsize);


            //image undistortion (if required)
           if(undistort ){
               cv::remap(in_image,auxImage,undistMap[0],undistMap[1],cv::INTER_CUBIC);
               in_image=auxImage;
               image_params.Distorsion.setTo(cv::Scalar::all(0));
           }

            // enhanceImage (more contrast)
            // enhanceImageBGR(in_image, in_image);

            currentFrameIndex = vcap.getNextFrameIndex() - 1;

            Fps.start();
            camPose_c2g=Slam->process(in_image, image_params, currentFrameIndex);
            Fps.stop();

            TimerDraw.start();
            // Slam->drawMatches(in_image);
            // char k = TheViewer.show(&Slam, in_image,"#" + std::to_string(currentFrameIndex) + " fps=" + to_string(1./Fps.getAvrg()) );
            char k =0;
            if(!cml["-noX"]) {
                // draw mesh
                if(!camPose_c2g.empty()){
                    cout << "CamPose: " << camPose_c2g << endl << "---" << endl;
                    drawMesh(in_image, linePoints, projectionMatrix * camPose_c2g);
                }

                // draw tags & points
                k = TheViewer.show(TheMap, in_image, camPose_c2g,"#" + std::to_string(currentFrameIndex)/* + " fps=" + to_string(1./Fps.getAvrg())*/ ,Slam->getCurrentKeyFrameIndex());
            }

            if (int(k) == 27 || k=='q') finish = true; //pressed ESC
            TimerDraw.stop();

            //save to output pose?
            if (toSaveCamPose) {
                if(camPose_c2g.empty()) {
                    outCamPose << currentFrameIndex << " ";
                    // pose[x, y, z] = (0, 0, 0) and quaternion[w, x, y, z] = (1, 0, 0, 0)
                    outCamPose << "0 0 0 1 0 0 0" << endl;
                } else {
                    auto camPoseQuaternion = rotationMatrixToQuaternion(camPose_c2g.rowRange(0, 3).colRange(0, 3));
                    outCamPose <<
                               currentFrameIndex << " " <<
                               camPose_c2g.at<float>(0, 3) << " " <<
                               camPose_c2g.at<float>(1, 3) << " " <<
                               camPose_c2g.at<float>(2, 3) << " " <<
                               camPoseQuaternion[0] << " " <<
                               camPoseQuaternion[1] << " " <<
                               camPoseQuaternion[2] << " " <<
                               camPoseQuaternion[3] << endl;
                }
            }

            //save to output video?
            if (!TheOutputVideo.empty()){
                auto image=TheViewer.getImage();
                if(!videoout.isOpened()){
                    videoout.open(TheOutputVideo, CV_FOURCC('X', '2', '6', '4'), stof(cml("-fps","30")),image.size()  , image.channels()!=1);
                    cout << "open!";
                }

                if(videoout.isOpened()) {
                    videoout.write(image);
                }
            }

            //reset?
            if (k=='r') Slam->clear();
            //write the current map
            if (k=='e'){
                string number = std::to_string(currentFrameIndex);
                while (number.size() < 5) number = "0" + number;
                TheMap->saveToFile("world-"+number+".map");
            }
            if (k=='v'){
                Slam->saveToFile("Slam->slm");
            }
            if(k=='s'){
                TheMap->saveToFile(cml("-out","world") +".map");
            }
            // if(k=='o'){
            //     TheMap->saveToFile(cml("-out","world") +".map");
            //     fullbaOptimization(*TheMap);
            //     TheMap->saveToFile(cml("-out","world") +".map");
            // }
                
            FpsComplete.stop();
            if(currentFrameIndex % 10 == 0){
                // TheMap->removeUnUsedKeyPoints();
                // TheMap->removeOldFrames();

                cout << "Image " << currentFrameIndex << " fps=" << 1./Fps.getAvrg()<<" "<<1./FpsComplete.getAvrg();
                cout << " draw=" << 1./TimerDraw.getAvrg();
                cout << (camPose_c2g.empty()?" not tracked":" tracked") << endl;
            }
        } catch (...) { //const std::exception &ex) {
            errorFlag = true;
            cerr << "an error occurs" << endl;

            if (cml["-isInstancing"]){
                delete Slam;
                Slam = new tslam::TSlam;
                TheMap = std::make_shared<tslam::Map>();
                if (cml["-map"]){
                    TheMap->readFromFile(cml("-map"));
                }

                Slam->setParams(TheMap, params, cml("-voc"));
            }

        }
        //read next
        vcap>>in_image;
        if(!camPose_c2g.empty()){
            for(int s=0;s<vspeed-1;s++)
                vcap >> in_image;
        }
    }

    //release the video output if required
    if(videoout.isOpened()) videoout.release();

    //close the output file
    if (toSaveCamPose) outCamPose.close();

    //optimize the map
    // fullbaOptimization(*TheMap);

    //save the output
    TheMap->saveToFile(cml("-out","world") +".map");
    //save also the parameters finally employed
    params.saveToYMLFile("tslam_params_"+cml("-out","world") +".yml");
    if (debugLevel >=10){
        Slam->saveToFile("Slam->slm");
    }
    TheMap->saveToMarkerMap("markermap.yml");


    // }
    // catch (const std::exception &ex) {
    //     errorFlag = true;
    //     cerr << ex.what() << endl;
    // }
    if (errorFlag) {
        cout << "Program ends with an error." << endl;
    }
}

// /home/tpp/Downloads/long-beam-1-480p-2.mp4 /home/tpp/UCOSlam-IBOIS/test_result/calibration_pixel_480p.yml -voc /home/tpp/UCOSlam-IBOIS/orb.fbow -out test.map -map /home/tpp/UCOSlam-IBOIS/build/utils/long1-px-480p-combine-compressed.map -isInstancing