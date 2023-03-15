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

cv::Mat resize(cv::Mat &in, cv::Size size){
    if (size.area() <= 0) return in;
    cv::Mat ret, ret2;
    cv::resize(in,ret,size);  return ret;
}

//////////////////////////////////
/// Command line config parser ///
//////////////////////////////////

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

void overwriteParamsByCommandLine(CmdLineParser &cml,tslam::Params &params){
    if (cml["-aruco-markerSize"])           params.aruco_markerSize = stof(cml("-aruco-markerSize", "1"));
    if (cml["-marker_minsize"])             params.aruco_minMarkerSize= stod(cml("-marker_minsize", "0.025"));
    if (cml["-nokeypoints"])                params.detectKeyPoints=false;
    if (cml["-nomarkers"])                  params.detectMarkers =false;
    if (cml["-sequential"])                 params.runSequential=true;
    if (cml["-maxFeatures"])                params.maxFeatures = stoi(cml("-maxFeatures","4000"));
    if (cml["-nOct"])                       params.nOctaveLevels = stoi(cml("-nOct","8"));
    if (cml["-fdt"])                        params.nthreads_feature_detector = stoi(cml("-fdt", "1"));
    if (cml["-desc"])                       params.kpDescriptorType = tslam::DescriptorTypes::fromString(cml("-desc", "orb"));
    if (cml["-dict"])                       params.aruco_Dictionary = cml("-dict");
    if (cml["-tfocus"])                     params.targetFocus =stof(cml("-tfocus","-1"));
    if (cml["-KFMinConfidence"])            params.KFMinConfidence =stof(cml("-KFMinConfidence"));
    if (cml["-nonmax"])                     params.KPNonMaximaSuppresion=true;
    if (cml["-saveImages"])                 params.saveImageInMap=true;
    if (cml["-isInstancing"])               params.isInstancing=true;
    if (cml["-autoAdjustKpSensitivity"])    params.autoAdjustKpSensitivity=true;
    if (cml["-extra_params"])               params.extraParams=cml("-extra_params");
    if (cml["-scale"])                      params.kptImageScaleFactor=stof(cml("-scale"));
    if (cml["-nokploopclosure"])            params.reLocalizationWithKeyPoints=false;
    if (cml["-inplanemarkers"])             params.inPlaneMarkers=true;

    params.aruco_CornerRefimentMethod=cml("-aruco-cornerRefinementM","CORNER_SUBPIX");

    if(cml["-enableLoopClosure"]) params.enableLoopClosure=true;
    else params.enableLoopClosure=false;

    if (cml["-dbg_str"]) tslam::debug::Debug::addString(cml("-dbg_str"),"");
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

    cv::Mat clahe_L;
    clahe->apply(Lab_planes[0], clahe_L);

    // Merge the color planes back into an Lab image
    clahe_L.copyTo(Lab_planes[0]);
    cv::merge(Lab_planes, Lab_image);

    // convert back to RGB
    cv::cvtColor(Lab_image, imgOut, cv::COLOR_Lab2BGR);
}

//////////////////////
/// Mesh Rendering ///
//////////////////////

cv::Mat projectionMatrix;

cv::Vec4f convertRotationMatrixToQuaternion(cv::Mat R) {
    cv::Vec4f q;
    q[0] = sqrt(1.0 + R.at<float>(0,0) + R.at<float>(1,1) + R.at<float>(2,2)) / 2.0;
    q[1] = (R.at<float>(2,1) - R.at<float>(1,2)) / (4.0 * q[0]);
    q[2] = (R.at<float>(0,2) - R.at<float>(2,0)) / (4.0 * q[0]);
    q[3] = (R.at<float>(1,0) - R.at<float>(0,1)) / (4.0 * q[0]);
    return q;
}

void loadPly(string path, vector<cv::Vec3f> &vertices, vector<cv::Vec3i> &faces, vector<pair<cv::Vec3f, cv::Vec3f>> &lines){
    int elementVertex = 0;
    int elementFace = 0;

    ifstream plyFile;
    plyFile.open(path);
    if(!plyFile.is_open()){
        cout << "Error opening file" << endl;
        return;
    }
    string line;
    size_t found;
    while(getline(plyFile, line)){
        found = line.find("element vertex");
        if(found != string::npos){
            auto num = line.substr(found + string("element vertex").length());
            elementVertex = stoi(num);
        }
        found = line.find("element face");
        if(found != string::npos){
            auto num = line.substr(found + string("element face").length());
            elementFace = stoi(num);
        }
        if(line == "end_header") break;
    }

    float x, y, z;
    for(int i = 0 ; i < elementVertex ; i++){
        getline(plyFile, line);
        stringstream ss(line);
        ss >> x >> y >> z;
        cv::Vec3f point(x, y, z);
        vertices.emplace_back(point);
    }

    int num, a, b, c;
    for(int i = 0 ; i < elementFace ; i++) {
        getline(plyFile, line);
        stringstream ss(line);
        ss >> num >> a >> b >> c;
        cv::Vec3i face(a, b, c);
        faces.emplace_back(face);

        cv::Vec3f pt1 = vertices[a];
        cv::Vec3f pt2 = vertices[b];
        cv::Vec3f pt3 = vertices[c];

        lines.emplace_back(make_pair(pt1, pt2));
        lines.emplace_back(make_pair(pt2, pt3));
        lines.emplace_back(make_pair(pt3, pt1));

    }
}

auto getZ(cv::Mat p){
    return p.at<float>(2, 0);
};

std::pair<cv::Point2f, cv::Point2f>projectToScreen(int imgW, int imgH, std::pair<cv::Vec3f, cv::Vec3f> ptsToDraw, cv::Mat projectionMatrix, cv::Mat cameraPose){
    std::pair<cv::Vec4f, cv::Vec4f> reprojecPts;

    float zNear = 1.0f; // clip plane is at (0, 0, -1) in camera space

    cv::Mat p1 = (cv::Mat_<float>(4, 1) << ptsToDraw.first[0], ptsToDraw.first[1], ptsToDraw.first[2], 1);
    p1 = cameraPose * p1;

    cv::Mat p2 = (cv::Mat_<float>(4, 1) << ptsToDraw.second[0], ptsToDraw.second[1], ptsToDraw.second[2], 1);
    p2 = cameraPose * p2;

    auto clipOnNearPlane = [&zNear](cv::Vec4f p1, cv::Vec4f p2) {
        auto ptFront = p1, ptBack = p2;

        // ptsToDraw.first is behind the camera
        if (p1[2] < zNear) {
            ptFront = p2;
            ptBack = p1;
        }

        float pz;

        ptBack = (abs(zNear - ptBack[2]) * ptFront + abs(zNear - ptFront[2]) * ptBack) / abs(ptFront[2] - ptBack[2]);
        ptBack[3] = 1.0f; // force w = 1

        return make_pair(ptBack, ptFront);
    };

    if( (getZ(p1) < zNear && getZ(p2) < zNear) ) { // both are behind the camera, no need to render
//        cout << "Both points are behind camera, no need to render." << endl;
        return std::make_pair(cv::Point2f(-1, -1), cv::Point2f(-1, -1));
    } else if ( !(getZ(p1) > zNear && getZ(p2) > zNear) ) {
//        cout << "Perform near clipping." << endl;
        reprojecPts = clipOnNearPlane(p1, p2);
    } else {
        reprojecPts.first = cv::Vec4f(p1.at<float>(0, 0), p1.at<float>(1, 0), p1.at<float>(2, 0), 1.0f);
        reprojecPts.second = cv::Vec4f(p2.at<float>(0, 0), p2.at<float>(1, 0), p2.at<float>(2, 0), 1.0f);
    }

    p1 = projectionMatrix * reprojecPts.first;
    p2 = projectionMatrix * reprojecPts.second;

    reprojecPts.first = cv::Vec4f(p1.at<float>(0, 0), p1.at<float>(1, 0), p1.at<float>(2, 0), p1.at<float>(3, 0));
    reprojecPts.second = cv::Vec4f(p2.at<float>(0, 0), p2.at<float>(1, 0), p2.at<float>(2, 0), p2.at<float>(3, 0));

    reprojecPts.first  /= reprojecPts.first[2];  // make all z = 1
    reprojecPts.second /= reprojecPts.second[2]; // make all z = 1

    int x1 = int(imgW - (reprojecPts.first[0] + 1.0f) / 2.0f * imgW);
    int y1 = int(imgH - (reprojecPts.first[1] + 1.0f) / 2.0f * imgH);

    int x2 = int(imgW - (reprojecPts.second[0] + 1.0f) / 2.0f * imgW);
    int y2 = int(imgH - (reprojecPts.second[1] + 1.0f) / 2.0f * imgH);

//    cout << "x1: " << x1 << " y1: " << y1 << " x2: " << x2 << " y2: " << y2 << endl;

    return make_pair(cv::Point2f(x1, y1), cv::Point2f(x2, y2));
}

void drawMesh(cv::Mat img, vector<pair<cv::Vec3f, cv::Vec3f>> linesToDraw, cv::Mat projectionMatrix, cv::Mat cameraPose) {
    int flag = 0;
    int count = 0;
    for(auto &ptsToDraw:linesToDraw) {
        auto ptsScreen = projectToScreen(img.cols, img.rows, ptsToDraw, projectionMatrix, cameraPose);
        cv::line(img, ptsScreen.first, ptsScreen.second, cv::Scalar(0, 0, 255), 2);
    }
}


////////////
/// Main ///
////////////

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

    bool isExportingVideo = !cml("-outvideo").empty();
    cv::VideoWriter outVideoWriter;
    cv::VideoWriter outRawVideoWriter;
    string inputVideo = string(argv[1]);
    string outputVideoPath = cml("-outvideo") + ".mp4";
    string outputRawVideoPath = cml("-outvideo") + "_raw.mp4";

    if (inputVideo.find("live") != std::string::npos)
    {
        int vIdx = 0;
        // check if the :idx is here
        char cad[100];
        if (inputVideo.find(":") != string::npos)
        {
            std::replace(inputVideo.begin(), inputVideo.end(), ':', ' ');
            sscanf(inputVideo.c_str(), "%s %d", cad, &vIdx);
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
    cout << "camW: " << camW << " camH: " << camH << endl;
    float x0 = 0, y0 = 0,zF = 10000.0f, zN =0.0001f;
    float fovX = cameraMatrix.at<float>(0,0);
    float fovY = cameraMatrix.at<float>(1,1);
    float cX = cameraMatrix.at<float>(0,2);
    float cY = cameraMatrix.at<float>(1,2);
    float perspectiveProjMatrixData[16] = {
            2.0f * fovX / camW,    0, ( camW - 2 * cX + 2 * x0) / camW,                         0,
            0,    2.0f * fovY / camH, (-camH + 2 * cY + 2 * y0) / camH,                         0,
            0,                  0,             (-zF - zN)/(zF - zN),  -2 * zF * zN / (zF - zN),
            0,                  0,                               -1,                         0
    };

    projectionMatrix = cv::Mat(4, 4, CV_32F, perspectiveProjMatrixData);

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

    if(!cml["-voc"]  && !cml["-map"]) {
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

    vector<cv::Vec3f> vertices;
    vector<cv::Vec3i> faces;
    vector<pair<cv::Vec3f, cv::Vec3f>> lines;
    if(cml["-drawMesh"]){
        loadPly(cml("-drawMesh"), vertices, faces, lines);

//        auto l0 = lines[3];
//        lines.clear();
//        lines.emplace_back(l0);

        // for test => only draw one line
//        lines.clear();
//        lines.emplace_back(
//                make_pair(
//                        cv::Vec3f(-4.3453914642333984e+01, -1.2498917579650879e+01, 3.6316684722900391e+01),
//                        cv::Vec3f(-4.4349357604980469e+01, -1.2652835845947266e+01, 3.6734409332275391e+01)
//                )
//        );
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

    cv::Mat auxImage;
    //Ok, lets start
    tslam::TimerAvrg Fps;
    tslam::TimerAvrg FpsComplete;
    tslam::TimerAvrg TimerDraw;
    bool finish = false;
    cv::Mat camPose_c2g;
    int vspeed = stoi(cml("-vspeed","1"));

    vector<cv::Mat> rawFramesToWrite;
    vector<cv::Mat> slamFramesToWrite;

    while (!finish && !in_image.empty())  {
        try{
            FpsComplete.start();

            if (isExportingVideo) {
                rawFramesToWrite.emplace_back(in_image.clone());
            }

            //image resize (if required)
            in_image = resize(in_image, vsize);

            //image undistortion (if required)
           if(undistort){
               cv::remap(in_image,auxImage,undistMap[0],undistMap[1],cv::INTER_CUBIC);
               in_image = auxImage;
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
                if(cml["-drawMesh"] && !camPose_c2g.empty()){
                    drawMesh(in_image, lines, projectionMatrix, camPose_c2g);
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
                    auto camPoseQuaternion = convertRotationMatrixToQuaternion(
                            camPose_c2g.rowRange(0, 3).colRange(0, 3));
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

            // save to output video?
            if (isExportingVideo) {
                slamFramesToWrite.emplace_back(TheViewer.getImage().clone());
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
            if(k=='c'){

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
        } catch (...) {
        // } catch (const std::exception &ex) {
            // cerr << ex.what() << endl;

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
        vcap >> in_image;
        if(!camPose_c2g.empty()){
            for(int s=0;s<vspeed-1;s++)
                vcap >> in_image;
        }
    }

    //release the video output if required
    if(isExportingVideo){
        cout << "Exporting Video..." << endl;
        bool errFlag = false;
        if(slamFramesToWrite.empty() || rawFramesToWrite.empty()){
            cerr << "No frames to write" << endl;
            errFlag = true;
        }
        if(!errFlag){
            outVideoWriter.open(
                    outputVideoPath,
                    CV_FOURCC('m', 'p', '4', 'v'),
                    stof(cml("-fps","10")),
                    slamFramesToWrite.at(0).size(), slamFramesToWrite.at(0).channels()!=1);
            outRawVideoWriter.open(
                    outputRawVideoPath,
                    CV_FOURCC('m', 'p', '4', 'v'),
                    stof(cml("-fps","10")),
                    rawFramesToWrite.at(0).size(), rawFramesToWrite.at(0).channels()!=1);

            for(auto image : slamFramesToWrite){
                outVideoWriter.write(image);
            }

            for(auto image : rawFramesToWrite){
                outRawVideoWriter.write(image);
            }

            outVideoWriter.release();
            outRawVideoWriter.release();
        }
    }


    //close the output file
    if (toSaveCamPose) outCamPose.close();

    //optimize the map
    if(cml["-optimizeMap"]) TheMap->optimize();

    //save the output
    TheMap->saveToFile(cml("-out","world") +".map");
    TheMap->saveToMarkerMap(cml("-out","world") +".yml");

    //save also the parameters finally employed
    params.saveToYMLFile("tslam_params_"+cml("-out","world") +".yml");
    if (debugLevel >=10){
        Slam->saveToFile("Slam->slm");
    }


    if (errorFlag) {
        cout << "Program ends with an error." << endl;
    }
}

// /home/tpp/Downloads/long-beam-1-480p-2.mp4 /home/tpp/UCOSlam-IBOIS/test_result/calibration_pixel_480p.yml -voc /home/tpp/UCOSlam-IBOIS/orb.fbow -out test.map -map /home/tpp/UCOSlam-IBOIS/build/utils/long1-px-480p-combine-compressed.map -isInstancing