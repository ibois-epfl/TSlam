/**
This file is part of TSLAM and distributed under the terms of the GPLv3.

The modified version: Copyright (C) 2023 Andrea Settimi <andrea dot settimi at epfl dot ch>, Hong-Bin Yang <hong dash bin dot yang at epfl dot ch> (IBOIS, EPFL)
The original project: Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas, MODIFIED BY Andrea Settimi and Hong-Bin Yang ''AS IS'' AND ANY EXPRESS OR IMPLIED
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
or implied, of Rafael Muñoz Salinas.
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
