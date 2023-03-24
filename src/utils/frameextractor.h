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
* along with TSLAM. If not, see <http://wwmap->gnu.org/licenses/>.
*/
#ifndef tslam_FrameExtractor_H
#define tslam_FrameExtractor_H
#include <vector>
#include <memory>
#include <opencv2/core/core.hpp>
#include "featureextractors/feature2dserializable.h"
#include "map_types/frame.h"
#include "imageparams.h"
#include "markerdetector.h"
#include <memory>
namespace  aruco{
class MarkerDetector;
};

namespace tslam {



/** This class process the input image(s) and creates the Frame that will be used for processing
  */
class FrameExtractor{
public:

    FrameExtractor();
    void setParams(std::shared_ptr<Feature2DSerializable>  feature_detector, const tslam::Params &params, std::shared_ptr<tslam::MarkerDetector> mdetector);

    void process(const cv::Mat &image, const ImageParams &ip,Frame &frame, uint32_t frameseq_idx=std::numeric_limits<uint32_t>::max() );
    void process_rgbd(const cv::Mat &image, const cv::Mat &depthImage,const ImageParams &ip,Frame &frame, uint32_t frameseq_idx=std::numeric_limits<uint32_t>::max());
    void processStereo(const cv::Mat &LeftRect, const cv::Mat &RightRect,const ImageParams &ip,Frame &frame, uint32_t frameseq_idx=std::numeric_limits<uint32_t>::max());

    //enables/disables removing keypoints into the markers
    bool &removeFromMarkers(){return _removeFromMarkers;}
    bool &detectMarkers(){return _detectMarkers;}
    bool &detectKeyPoints(){return _detectKeyPoints;}


     void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);

    void setSensitivity(float v);
    float  getSensitivity();

private:
    //obfuscate start
    struct ImgInfo{
        cv::Mat im_org,im_resized;
        ImageParams ip_org,ip_resized;
        pair<float,float> scaleFactor;
    };
    std::shared_ptr<Feature2DSerializable> _fdetector;

    void preprocessImages(float scaleFactor,const cv::Mat &im1,const ImageParams &ip,const cv::Mat &im2=cv::Mat());
    void extractFrame(const ImgInfo& Iinfo,  Frame &frame, uint32_t frameseq_idx=std::numeric_limits<uint32_t>::max());


     vector<ImgInfo> InputImages;
    //cv::Mat _imgrey,_imgreyDownsampled;
    uint32_t _counter=0;//used to assign a unique id to each processed frame
    bool _removeFromMarkers=true;
    bool _detectMarkers=true;
    bool _detectKeyPoints=true;

    Feature2DSerializable::FeatParams _featParams;
    std::shared_ptr<tslam::MarkerDetector> _mdetector;
    float _markerSize=0 ;
    float _maxDescDist=0;
    tslam::Params _tslamParams;
    //obfuscate end

};

}
#endif
