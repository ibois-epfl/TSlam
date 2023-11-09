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
#ifndef TSLAM_GRIDEXTRACTOR_H
#define TSLAM_GRIDEXTRACTOR_H
#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include "feature2dserializable.h"
namespace tslam
{


//divide the image in a grid of patches and computes the features in them independently
class GridExtractor:public Feature2DSerializable{
public:

    //nParts for each axis
    void setParams(F2S_Type type );
    void detectAndCompute_impl( cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints,
                           cv::OutputArray descriptors,FeatParams params );
    void toStream_impl(std::ostream &str);
    void fromStream_impl(std::istream &str);
    F2S_Type getType()const{return _ftype;}


    Feature2DSerializable::FeatParams getParams()const{return _featParams;}

    virtual float getMinDescDistance()const;
    DescriptorTypes::Type getDescriptorType()const;
private:
    void  detectAndCompute_thread(cv::InputArray _image, cv::InputArray _mask, cv::Ptr<cv::Feature2D> extractor, cv::Ptr<cv::Feature2D> detector, std::vector<cv::KeyPoint> &vkeypoints,  cv::Mat &vdesc, int y);

    void  createDetectors(Feature2DSerializable::FeatParams param,cv::Size imageSize);

     Feature2DSerializable::FeatParams _featParams;
    F2S_Type _ftype;
     std::vector<cv::Ptr<cv::Feature2D> >_fextractors;
    std::vector<cv::Ptr<cv::Feature2D> >_fdetectors;
     bool _areParamsSet=false;
    int _overlapborder=0;
};

}

#endif
