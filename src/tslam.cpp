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
cv::Mat TSlam::process(  cv::Mat &in_image,const ImageParams &ip,uint32_t frameseq_idx){
    if(in_image.size()!=ip.CamSize)
        throw  std::runtime_error("Input Image Size is Different from Image Params");
    return  reinterpret_cast<System*>(impl)->process(in_image,ip,frameseq_idx);
}
cv::Mat TSlam::processStereo( cv::Mat &in_image,const cv::Mat &R_image,const ImageParams &ip,uint32_t frameseq_idx){
    return  reinterpret_cast<System*>(impl)->process(in_image,ip,frameseq_idx,cv::Mat(),R_image);
}
cv::Mat TSlam::processRGBD( cv::Mat &in_image,const cv::Mat & depth,const ImageParams &ip,uint32_t frameseq_idx){
    return  reinterpret_cast<System*>(impl)->process(in_image,ip,frameseq_idx,depth);
}
cv::Mat TSlam::processArray(vector<cv::Mat> &images,  ImageParams &ArrayCamParams ,uint32_t frameseq_idx)
{
    return  reinterpret_cast<System*>(impl)->process(images, ArrayCamParams, frameseq_idx);
}
void TSlam::resetTracker(){
    reinterpret_cast<System*>(impl)->resetTracker();
}
void TSlam::setMode(MODES mode){
    reinterpret_cast<System*>(impl)->setMode(mode);
}
//void TSlam::resetCurrentPose(){
//    reinterpret_cast<System*>(impl)->resetCurrentPose();
//}
uint32_t TSlam::getLastProcessedFrame()const{
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
    
    map = std::make_shared<tslam::Map>();
    updateSystem();
}

void TSlam::setMap(std::string pathToMapFile){
    // auto newMap = std::shared_ptr<Map> newMap;
    auto newMap = std::make_shared<tslam::Map>();
    newMap->readFromFile(pathToMapFile);
    map = newMap;
    updateSystem();
}

void TSlam::setVocabulary(std::string pathToVocFile){
    this->pathToVoc = pathToVocFile;
    updateSystem();
}

void TSlam::setCamParams(std::string pathToCamParamFile){
    imageParams.readFromXMLFile(pathToCamParamFile);
}

void TSlam::setInstancing(bool isInstancing){
    systemParams.isInstancing = isInstancing;
    auto newMap = map;
    map = newMap;
    updateSystem();
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
    delete (System*)impl;
    impl = new System;
    setParams(map, systemParams, pathToVoc);
}

}
