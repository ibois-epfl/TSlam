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

// header for fullba
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include <iostream>
#include <exception>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <type_traits>
#include "cvprojectpoint.h"
#include "g2oba.h"

#include "ucoslam.h"
#include "basictypes/debug.h"
#include "mapviewer.h"
#include "basictypes/timers.h"
#include "map.h"
#include "inputreader.h"
#include "basictypes/cvversioning.h"


cv::Mat getImage(cv::VideoCapture &vcap,int frameIdx){
    cv::Mat im;
    ucoslam::Frame frame;
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


void overwriteParamsByCommandLine(CmdLineParser &cml,ucoslam::Params &params){
    if ( cml["-aruco-markerSize"])      params.aruco_markerSize = stof(cml("-aruco-markerSize", "1"));
    if ( cml["-marker_minsize"])    params.aruco_minMarkerSize= stod(cml("-marker_minsize", "0.025"));
    if (cml["-nokeypoints"])params.detectKeyPoints=false;
    if (cml["-nomarkers"])  params.detectMarkers =false;
    if (cml["-sequential"]) params.runSequential=true;
    if (cml["-maxFeatures"])    params.maxFeatures = stoi(cml("-maxFeatures","4000"));
    if (cml["-nOct"])       params.nOctaveLevels = stoi(cml("-nOct","8"));
    if (cml["-fdt"])        params.nthreads_feature_detector = stoi(cml("-fdt", "2"));
     if (cml["-desc"])       params.kpDescriptorType = ucoslam::DescriptorTypes::fromString(cml("-desc", "orb"));
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

    if (cml["-dbg_str"])
        ucoslam::debug::Debug::addString(cml("-dbg_str"),"");
}

std::pair<std::string,std::string> getSplit(std::string str){
    std::string livestr=str;
    for(auto &c:livestr)if(c==':')c=' ';
    std::stringstream sstr;sstr<<livestr;
    std::string aux,aux2;int n=0;
    sstr>>aux>>aux2;
    return std::pair<std::string,std::string>(aux,aux2);
};

void fullbaOptimization(ucoslam::Map &TheMap){
    std::cout << "Optimizing Map..." << std::endl;
    try {
        
        std::shared_ptr<g2o::SparseOptimizer> Optimizer;

        Optimizer=std::make_shared<g2o::SparseOptimizer>();
        std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver=g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));


        Optimizer->setAlgorithm(solver);

        int id=0;
        //first, the cameras
        g2oba::CameraParams * camera = new g2oba::CameraParams();
        camera->setId(id++);



        //////////////////////////////
        ///KEYFRAME POSES
        //////////////////////////////
        map<uint32_t, g2oba::SE3Pose * > kf_poses;
        for(auto &kf:TheMap.keyframes){
            if(!camera->isSet()){
                camera->setParams(kf.imageParams.CameraMatrix,kf.imageParams.Distorsion);
                Optimizer->addVertex(camera);
            }
            g2oba::SE3Pose * pose= new g2oba::SE3Pose(kf.idx, kf.pose_f2g.getRvec(),kf.pose_f2g.getTvec());
            pose->setId(id++);
            Optimizer->addVertex(pose);
            kf_poses.insert({kf.idx,pose});
        }
        //////////////////////////////
        ///MAP POINTS
        //////////////////////////////
        list<g2oba::ProjectionEdge *> projectionsInGraph;
        list<g2oba::MapPoint *> mapPoints;
        for(auto &p:TheMap.map_points){
            g2oba::MapPoint *point=new g2oba::MapPoint (p.id, p.getCoordinates());
            point->setId(id++);
            point->setMarginalized(true);
            Optimizer->addVertex(point);
            mapPoints.push_back(point);

            for(auto of:p.getObservingFrames()){
                auto &kf=TheMap.keyframes[of.first];
                if ( kf.isBad() )continue;

                auto Proj=new g2oba::ProjectionEdge(p.id,kf.idx, kf.kpts[of.second]);
                Proj->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(point));
                Proj->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( camera));
                Proj->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>( kf_poses.at(kf.idx)));
                Eigen::Matrix<double,2,1> obs;
                obs<<kf.kpts[of.second].x,kf.kpts[of.second].y;
                Proj->setMeasurement(obs);
                Proj->setInformation(Eigen::Matrix2d::Identity()* 1./ kf.scaleFactors[kf.und_kpts[of.second].octave]);
                g2o::RobustKernelHuber* rk = new  g2o::RobustKernelHuber();
                rk->setDelta(sqrt(5.99));
                Proj->setRobustKernel(rk);
                Optimizer->addEdge(Proj);
                projectionsInGraph.push_back(Proj);
            }
        }

        //////////////////////////////
        ////MARKER POSES
        //////////////////////////////
        map<uint32_t, g2oba::SE3Pose * > marker_poses;
        std::vector<g2oba::MarkerEdge * > marker_edges;
        for(const auto &marker:TheMap.map_markers){
            if(!marker.second.pose_g2m.isValid()) continue;
            g2oba::SE3Pose * marker_pose= new g2oba::SE3Pose(marker.first,marker.second.pose_g2m.getRvec(),marker.second.pose_g2m.getTvec());
            marker_pose->setId(id++);
            Optimizer->addVertex(marker_pose);
            marker_poses.insert({marker.first,marker_pose});

            for(const auto &kfidx:marker.second.frames){
                auto &kf=TheMap.keyframes[kfidx];
                if ( kf.isBad() )continue;
                if(  kf_poses.count(kfidx)==0)throw std::runtime_error("Key frame for marker not in the optimization:"+std::to_string(kfidx));

                auto Proj=new g2oba::MarkerEdge(marker.second,kfidx);
                Proj->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(marker_pose));
                Proj->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( kf_poses.at(kfidx)));
                Proj->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>( camera));
                auto mobs=TheMap.keyframes[kfidx].getMarker(marker.first);
                Eigen::Matrix<double,8,1> obs;
                obs<<mobs.corners[0].x,mobs.corners[0].y,
                        mobs.corners[1].x,mobs.corners[1].y,
                        mobs.corners[2].x,mobs.corners[2].y,
                        mobs.corners[3].x,mobs.corners[3].y;

                Proj->setMeasurement(obs);
                Proj->setInformation(Eigen::Matrix< double, 8, 8 >::Identity());
                g2o::RobustKernelHuber* rk = new  g2o::RobustKernelHuber();
                rk->setDelta(sqrt(15.507));
                Proj->setRobustKernel(rk);
                Optimizer->addEdge(Proj);
                marker_edges.push_back(Proj);
            }
        }





        //////////////////////////////
        /// OPTIMIZE
        //////////////////////////////

        int niters=50;
        Optimizer->initializeOptimization();
        //    Optimizer->setForceStopFlag( );
        Optimizer->setVerbose(true);
        Optimizer->optimize(niters,1e-3);
        //now remove outliers
        for(auto &p:projectionsInGraph)
            p->setLevel(p->chi2()>5.99);
        for(auto &p:marker_edges)
            p->setLevel(p->chi2()>15.507);

        Optimizer->optimize(niters,1e-4);

        //copy data back to the map
        //move data back to the map
        for(auto &mp: mapPoints ){
            TheMap.map_points[ mp->getid()].setCoordinates(mp->getPoint3d());
        }

        //now, the keyframes
        for(auto pose:kf_poses){
            TheMap.keyframes[pose.first].pose_f2g=ucoslam::se3((*pose.second)(0),(*pose.second)(1),(*pose.second)(2),(*pose.second)(3),(*pose.second)(4),(*pose.second)(5));
            TheMap.keyframes[pose.first].imageParams.CameraMatrix.at<float>(0,0)=camera->fx();
            TheMap.keyframes[pose.first].imageParams.CameraMatrix.at<float>(1,1)=camera->fy();
            TheMap.keyframes[pose.first].imageParams.CameraMatrix.at<float>(0,2)=camera->cx();
            TheMap.keyframes[pose.first].imageParams.CameraMatrix.at<float>(1,2)=camera->cy();
            for(int p=0;p<5;p++)
                TheMap.keyframes[pose.first].imageParams.Distorsion.ptr<float>(0)[p]=camera->dist()[p];
        }
//        //remove weak links
//        for(auto &p:projectionsInGraph){
//            if(p->chi2()>5.99) ucoslam::DebugTest::removeMapPointObservation(TheMap,p->point_id,p->frame_id);
//        }

        //finally, markers
        for(auto pose:marker_poses)
            TheMap.map_markers[pose.first].pose_g2m=ucoslam::se3((*pose.second)(0),(*pose.second)(1),(*pose.second)(2),(*pose.second)(3),(*pose.second)(4),(*pose.second)(5));


        cout<<"Final Camera Params "<<endl;
        cout<<TheMap.keyframes.begin()->imageParams.CameraMatrix<<endl;
        cout<<TheMap.keyframes.begin()->imageParams.Distorsion<<endl;

    } catch (const std::exception &ex) {
        std::cerr<<ex.what()<<std::endl;
    }
}

void enhanceImageBGR(const cv::Mat &imgIn, cv::Mat &imgOut){
    // Start CLAHE Contrast Limited and Adaptive Histogram Equalization

    //Get Intesity image
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

int main(int argc,char **argv){
    bool errorFlag = false;
	// try {

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

        ucoslam::UcoSlam Slam;
		int debugLevel = stoi(cml("-debug", "0"));
        Slam.setDebugLevel(debugLevel);
        Slam.showTimers(true);
        ucoslam::ImageParams image_params;
        ucoslam::Params params;
        cv::Mat in_image;

        image_params.readFromXMLFile(argv[2]);

        if( cml["-params"])        params.readFromYMLFile(cml("-params"));
        overwriteParamsByCommandLine(cml,params);

        auto TheMap=std::make_shared<ucoslam::Map>();
        //read the map from file?
        if ( cml["-map"]) TheMap->readFromFile(cml("-map"));
        
        ///////////////////////////////////////////
        // Read from the text file
        // ifstream yamlFile("/home/tpp/UCOSlam-IBOIS/build/utils/merged_map.yml");
        // string yamlString;
        // string tmpStr;
        // while (getline(yamlFile, tmpStr)) {
        //     yamlString += tmpStr;
        // }

        // map<int, vector<vector<double> > > markers;

        // int index = yamlString.find("corners");
        // int markerAmount = 95;
        // for(int mi = 0; mi < markerAmount; mi++) {
        //     vector<vector<double> > markerCorners;
        //     for(int ci = 0; ci < 4; ci++){
        //         vector<double> markerCorner(3);
        //         for(int vi = 0; vi < 3; vi++){
        //             markerCorner[vi] = getDouble(yamlString, index);
        //         }
        //         markerCorners.push_back(markerCorner);
        //     }
        //     int id = getInt(yamlString, index);
        //     markers[id] = markerCorners;
        // }
        ////////////////////////////////////////////////



        Slam.setParams(TheMap, params,cml("-voc"));

        if(!cml["-voc"]  && !cml["-map"])
        {
            cerr<<"Warning!! No VOCABULARY INDICATED. KEYPOINT RELOCALIZATION IMPOSSIBLE WITHOUT A VOCABULARY FILE!!!!!"<<endl;
        }


        if (cml["-loc_only"]) Slam.setMode(ucoslam::MODE_LOCALIZATION);

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
        //need to resize input image?
        cv::Size vsize(0,0);
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
        ucoslam::MapViewer TheViewer;

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
            Slam.setMode(ucoslam::MODE_LOCALIZATION);



        cv::Mat auxImage;
        //Ok, lets start
        ucoslam::TimerAvrg Fps;
        ucoslam::TimerAvrg FpsComplete;
        ucoslam::TimerAvrg TimerDraw;
        bool finish = false;
        cv::Mat camPose_c2g;
        int vspeed=stoi(cml("-vspeed","1"));
        while (!finish && !in_image.empty()) {
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

                enhanceImageBGR(in_image, in_image);

                int currentFrameIndex = vcap.getNextFrameIndex()-1;

                Fps.start();
                camPose_c2g=Slam.process(in_image, image_params,currentFrameIndex);
                Fps.stop();


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
                if (k=='v'){
                    Slam.saveToFile("slam.slm");
                }
                if(k=='s'){
                    TheMap->saveToFile(cml("-out","world") +".map");
                }
                if(k=='o'){
                    TheMap->saveToFile(cml("-out","world") +".map");
                    TheMap->removeOldKeyPoints();
                    fullbaOptimization(*TheMap);
                    TheMap->saveToFile(cml("-out","world") +".map");
                }
                    
                FpsComplete.stop();
                if(currentFrameIndex % 100 == 0){
                    // TheMap->removeUnUsedKeyPoints();
                    // TheMap->removeOldFrames();

                    cout << "Image " << currentFrameIndex << " fps=" << 1./Fps.getAvrg()<<" "<<1./FpsComplete.getAvrg();
                    cout << " draw=" << 1./TimerDraw.getAvrg();
                    cout << (camPose_c2g.empty()?" not tracked":" tracked") << endl;
                }
            } catch (const std::exception &ex) {
                errorFlag = true;
                cerr << ex.what() << endl;
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

        //optimize the map
        // fullbaOptimization(*TheMap);

        //save the output
        TheMap->saveToFile(cml("-out","world") +".map");
        //save also the parameters finally employed
        params.saveToYMLFile("ucoslam_params_"+cml("-out","world") +".yml");
        if (debugLevel >=10){
            Slam.saveToFile("slam.slm");
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
