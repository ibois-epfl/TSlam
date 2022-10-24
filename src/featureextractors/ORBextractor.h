
#ifndef TSLAM_ORBEXTRACTOR_H
#define TSLAM_ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/features2d/features2d.hpp>


#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "feature2dserializable.h"

namespace tslam
{

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractor: public  Feature2DSerializable
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };
    ORBextractor();

    ~ORBextractor(){}
    void setNumberOfThreads(int nt);

    void detectAndCompute_impl( cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints,
                     cv::OutputArray descriptors, Feature2DSerializable::FeatParams params  );
    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    //serialization routines
    void toStream_impl(std::ostream &str);
    void fromStream_impl(std::istream &str);
    F2S_Type getType()const{return Feature2DSerializable::F2D_ORB;}

    float getMinDescDistance()const{return 50;}

    Feature2DSerializable::FeatParams getParams()const{return _featParams;}


    DescriptorTypes::Type getDescriptorType()const{return DescriptorTypes::DESC_ORB;}

    bool & doGaussianBlur(){return _doGaussianBlurAtFirst;}
    void setSensitivity(float v);
    float getSensitivity(float v);

protected:


    std::vector<cv::Mat> mvImagePyramid;
    bool _doGaussianBlurAtFirst=true;

    void compute( cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints, cv::OutputArray descriptors);
    void precalculateParams(Feature2DSerializable::FeatParams params,cv::Size imageSize);
    void ComputePyramid(cv::Mat image,const std::vector<int> &levels={});
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX, const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);
     void ComputeKeyPoints_thread(  std::vector<std::vector<cv::KeyPoint>  > *allKeypoints,  std::vector<int> vlevels);
     std::vector<cv::Point> pattern;

    Feature2DSerializable::FeatParams _featParams;

    int iniThFAST;
    int minThFAST;
    int maxFeatures;//temporary value used internally
    bool areParamsPrecomputed=false;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    std::vector<std::vector<int> >  _thread_levels;//levels assigned to each thread
    std::vector<std::vector<int> > assignLevelsToThreads(int nthreads,int nlevels);


    void processLevel(int level );
    void processLevel_thread(int id);
     //
 cv::Mat in_image;
 std::vector < std::vector<cv::KeyPoint> > _allKeypoints;
 std::vector<cv::Mat> _allDescriptors;
//thread sage queue
template <typename T>
class Queue
{
 public:

  T pop()
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
      cond_.wait(mlock);
    }
    auto item = queue_.front();
    queue_.pop();
    return item;
  }

  void push(const T& item)
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push(item);
    mlock.unlock();
    cond_.notify_one();
  }

 private:
  std::queue<T> queue_;
  std::mutex mutex_;
  std::condition_variable cond_;
};
Queue<int> ts_level_queue;
//not saved yet to stream
bool nonMaximaSuppression=false;

};

} //namespace ORB_SLAM

#endif

