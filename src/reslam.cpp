#include "reslam.h"
#include "utils/system.h"
#include "basictypes/debug.h"
namespace reslam{
ReSlam::ReSlam(std::shared_ptr<MarkerDetector> mdetector){
    impl=new System(mdetector);
}
ReSlam::~ReSlam(){
    delete (System*)impl;
}
void ReSlam::setParams(std::shared_ptr<Map> map, const  reslam::Params &params, const std::string &vocabulary){
    reinterpret_cast<System*>(impl)->setParams(map,params,vocabulary,nullptr);
}
void ReSlam::clear(){
    reinterpret_cast<System*>(impl)->clear();
}
Params  & ReSlam::getParams() {
    return System::getParams();
}
cv::Mat ReSlam::process(  cv::Mat &in_image,const ImageParams &ip,uint32_t frameseq_idx){
    if(in_image.size()!=ip.CamSize)
        throw  std::runtime_error("Input Image Size is Different from Image Params");
    vector<cv::Mat> images(1);
    images[0]=in_image;
    reinterpret_cast<System*>(impl)->setCamera(MONOCULAR);
    return  reinterpret_cast<System*>(impl)->process(images, ip, frameseq_idx);
}
cv::Mat ReSlam::processStereo( cv::Mat &in_image,const cv::Mat &R_image,const ImageParams &ip,uint32_t frameseq_idx){
    vector<cv::Mat> images(2);
    images[0]=in_image;
    images[1]=R_image;
    reinterpret_cast<System*>(impl)->setCamera(STEREO);
    return  reinterpret_cast<System*>(impl)->process(images, ip, frameseq_idx);
}
cv::Mat ReSlam::processRGBD( cv::Mat &in_image,const cv::Mat & depth,const ImageParams &ip,uint32_t frameseq_idx){
    vector<cv::Mat> images(2);
    images[0]=in_image;
    images[1]=depth;
    reinterpret_cast<System*>(impl)->setCamera(RGBD);
    return  reinterpret_cast<System*>(impl)->process(images,ip,frameseq_idx);
}
cv::Mat ReSlam::processArray(vector<cv::Mat> &images,  ImageParams &ArrayCamParams ,uint32_t frameseq_idx)
{
    reinterpret_cast<System*>(impl)->setCamera(ARRAY);
    return  reinterpret_cast<System*>(impl)->process(images, ArrayCamParams, frameseq_idx);
}
void ReSlam::resetTracker(){
    reinterpret_cast<System*>(impl)->resetTracker();
}
void ReSlam::backwardTracker(const cv::Mat pose_f2g, const std::vector<cv::Mat> &images, const ImageParams &img_params,uint32_t frameseq_idx){
    reinterpret_cast<System*>(impl)->backwardTracker(pose_f2g, images, img_params, frameseq_idx);
}
void ReSlam::setMode(MODES mode){
    reinterpret_cast<System*>(impl)->setMode(mode);
}
STATE ReSlam::getState(){
    return reinterpret_cast<System*>(impl)->getState();
}
//void ReSlam::resetCurrentPose(){
//    reinterpret_cast<System*>(impl)->resetCurrentPose();
//}
uint32_t ReSlam::getLastProcessedFrame()const{
    return  reinterpret_cast<System*>(impl)->getLastProcessedFrame();
}
void ReSlam::saveToFile(std::string filepath){
    reinterpret_cast<System*>(impl)->saveToFile(filepath);
}
void ReSlam::readFromFile(std::string filepath){
    reinterpret_cast<System*>(impl)->readFromFile(filepath);
}
void ReSlam::globalOptimization(){
    reinterpret_cast<System*>(impl)->globalOptimization();
}
//waits for all threads to finish
void ReSlam::waitForFinished(){
    reinterpret_cast<System*>(impl)->waitForFinished();
}
uint32_t ReSlam::getCurrentKeyFrameIndex(){
    return reinterpret_cast<System*>(impl)->getCurrentKeyFrameIndex();
}
std::shared_ptr<Map> ReSlam::getMap(){
    return reinterpret_cast<System*>(impl)->getMap();
}
void ReSlam::setDebugLevel(int level){
    debug::Debug::setLevel(level);
}
void ReSlam::showTimers(bool v){
    debug::Debug::showTimer(v);
}
void ReSlam::updateParams(const Params &p){
}
std::string ReSlam::getSignatureStr()const{
    return reinterpret_cast<System*>(impl)->getSignatureStr();
}
}
