/**
This file is part of TSLAM and distributed under the terms of the GPLv3.

Copyright (C) 2023 Andrea Settimi <andrea dot settimi at epfl dot ch>, Hong-Bin Yang <hong dash bin dot yang at epfl dot ch> (IBOIS, EPFL)

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Andrea Settimi and Hong-Bin Yang ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied Andrea Settimi and Hong-Bin Yang.
*/
#include <regex>

#include "tslam.h"
#include "utils/system.h"
#include "basictypes/debug.h"

namespace tslam{
    TSlam::TSlam(){
        impl=new System;
    }
    TSlam::~TSlam(){
        delete (System*)impl;
    }
    void TSlam::setParams(std::shared_ptr<Map> map, const  tslam::Params &params, const std::string &vocabulary){
        reinterpret_cast<System*>(impl)->setParams(map,params,vocabulary);
    }
    void TSlam::clear(){
        reinterpret_cast<System*>(impl)->clear();
    }
    Params  & TSlam::getParams() {
        return System::getParams();
    }
    cv::Mat TSlam::process(cv::Mat &in_image, const ImageParams &ip, uint32_t frameseq_idx){
        updateSystem();
        if(in_image.size()!=ip.CamSize)
            throw  std::runtime_error("Input Image Size is Different from Image Params");
        return  reinterpret_cast<System*>(impl)->process(in_image,ip,frameseq_idx);
    }
    void TSlam::resetTracker(){
        reinterpret_cast<System*>(impl)->resetTracker();
    }
    void TSlam::setMode(MODES mode){
        reinterpret_cast<System*>(impl)->setMode(mode);
    }
   
    Frame TSlam::getLastProcessedFrame()const{
        return reinterpret_cast<System*>(impl)->getLastProcessedFrame();
    }
    void TSlam::saveToFile(std::string filepath){
        reinterpret_cast<System*>(impl)->saveToFile(filepath);
    }
    void TSlam::readFromFile(std::string filepath){
        reinterpret_cast<System*>(impl)->readFromFile(filepath);
    }
    void TSlam::globalOptimization(){
        reinterpret_cast<System*>(impl)->globalOptimization();
    }
//waits for all threads to finish
    void TSlam::waitForFinished(){
        reinterpret_cast<System*>(impl)->waitForFinished();
    }
    uint32_t TSlam::getCurrentKeyFrameIndex(){
        return reinterpret_cast<System*>(impl)->getCurrentKeyFrameIndex();
    }
// std::shared_ptr<Map> TSlam::getMap(){
//     return reinterpret_cast<System*>(impl)->getMap();
// }
    void TSlam::setDebugLevel(int level){
        debug::Debug::setLevel(level);
    }
    void TSlam::showTimers(bool v){
        debug::Debug::showTimer(v);
    }
    void TSlam::updateParams(const Params &p){
    }
    std::string TSlam::getSignatureStr()const{
        return reinterpret_cast<System*>(impl)->getSignatureStr();
    }

/**
* Things the original UcoSlam doesn't implement
**/
    void TSlam::clearMap(){
        toClearMap = true;
    }

    void TSlam::setMap(std::string pathToMapFile, bool updateSystemImmediately){
        pathToMap = pathToMapFile;
        toUpdateSystem = true;
        toResetMap = true;
        if(updateSystemImmediately)
            updateSystem();
    }

    void TSlam::setVocabulary(std::string pathToVocFile){
        this->pathToVoc = pathToVocFile;
        toUpdateSystem = true;
    }

    void TSlam::setCamParams(std::string pathToCamParamFile){
        imageParams.readFromXMLFile(pathToCamParamFile);
    }

    void TSlam::setInstancing(bool localizeOnly){
        systemParams.localizeOnly = localizeOnly;
        toUpdateSystem = true;
    }

    bool TSlam::process(cv::Mat frame, cv::Mat &camPose){
        bool isTracked;
        camPose = process(frame, imageParams, currentFrameIndex++);
        if(camPose.empty()){
            isTracked = false;
            camPose = cv::Mat::eye(4, 4, CV_32F);
        } else {
            isTracked = true;
            camPose.copyTo(lastTrackedCamPose);
        }
        return isTracked;
    }

    void TSlam::updateSystem(){
        if(toUpdateSystem | toResetMap | toClearMap){
            delete (System*)impl;
            impl = new System;

            if(toResetMap){
                auto newMap = std::make_shared<tslam::Map>();
                newMap->readFromFile(pathToMap);
                map = newMap;
                toResetMap = false;
            }

            if(toClearMap){
                map = std::make_shared<tslam::Map>();
                toClearMap = false;
            }

            setParams(map, systemParams, pathToVoc);
            toUpdateSystem = false;
        }




    }

    void TSlam::CombineMap(string mapPathA, string mapPathB, string outputPath,
                           bool exportYml, bool exportPly,
                           ImageParams *estimatedImageParam, int niters) {

        std::shared_ptr<tslam::Map> TheMapA, TheMapB;
        TheMapA = std::make_shared<tslam::Map>();
        TheMapB = std::make_shared<tslam::Map>();

        TheMapA->readFromFile(std::move(mapPathA));
        TheMapB->readFromFile(std::move(mapPathB));

        TheMapA->merge(TheMapB);

        TheMapA->optimize(niters);

        if(estimatedImageParam != nullptr){
            *estimatedImageParam = TheMapA->keyframes.begin()->imageParams;
        }

        string basePath = outputPath;
        if(std::regex_match(basePath, std::regex("\\.map$"))){
            basePath.substr(0, basePath.length() - 4);
        }
        if(exportYml){
            TheMapA->saveToMarkerMap(basePath + ".yml");
        }
        if(exportPly){
            TheMapA->exportToFile(basePath + ".ply",cv::Scalar(125,125,125),cv::Scalar(255,0,0),cv::Scalar(0,0,255),{1111,1195,1129,1196,1141},cv::Scalar(0,255,0));
        }

        TheMapA->saveToFile(std::move(outputPath));
    }

    bool TSlam::Reconstruct3DModelAndExportPly(const std::string importTagMapPath,
                                               const std::string exportPlyPath,
                                               Reconstruct3DParams params) {
        return Reconstruct3DModelAndExportPly(importTagMapPath, exportPlyPath,
                                              params.radiusSearch,
                                              params.creaseAngleThreshold,
                                              params.minClusterSize,
                                              params.maxPlnDist,
                                              params.maxPlnAngle,
                                              params.aabbScaleFactor,
                                              params.maxPolyDist,
                                              params.eps);
    }

    bool TSlam::Reconstruct3DModelAndExportPly(const std::string importTagMapPath,
                                                const std::string exportPlyPath,
                                                float radiusSearch,
                                                double creaseAngleThreshold,
                                                int minClusterSize,
                                                double maxPlnDist,
                                                double maxPlnAngle,
                                                double aabbScaleFactor,
                                                double maxPolyDist,
                                                double eps){
        tslam::Reconstruction::TSLAMReconstructor reconstructor =
                tslam::Reconstruction::TSLAMReconstructor(
                    radiusSearch,
                    creaseAngleThreshold,
                    minClusterSize,
                    maxPlnDist,
                    maxPlnAngle,
                    aabbScaleFactor,
                    maxPolyDist,
                    eps
                );
        reconstructor.loadMap(importTagMapPath);
        reconstructor.run();

        // check the inner values of the geometric solver
        auto internGeoSolver = reconstructor.getGeometricSolver();

        if(internGeoSolver.checkMeshSanity() == false){
            return false;
        }

        try {
            reconstructor.saveMeshAsPLY(exportPlyPath);
            return true;
        } catch (std::exception &e) {
            std::cout << "Error: " << e.what() << std::endl;
            return false;
        }
    }

}
