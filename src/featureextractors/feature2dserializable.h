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

#ifndef TSLAM_FEATURE2DSerializable_H
#define TSLAM_FEATURE2DSerializable_H
#include <memory>
#include <opencv2/features2d/features2d.hpp>
#include "tslamtypes.h"


namespace tslam{

/**Class that allows serialization of the feature extractor. Is reimplemented by all feature extractors designed
 */
class   Feature2DSerializable{
public:


    struct FeatParams{
        int nthreads=-1;//auto
        int maxFeatures=4000;
        int nOctaveLevels=8;
        float scaleFactor=1.2;
        float sensitivity=0;

        FeatParams(){}
        FeatParams(int MaxFeatures,int NOctaveLevels,float ScaleFactor,int NThreads){
            maxFeatures=MaxFeatures;
            nOctaveLevels=NOctaveLevels;
            scaleFactor=ScaleFactor;
            nthreads=NThreads;
        }
        bool operator==(const FeatParams &fp){return nthreads==fp.nthreads && maxFeatures==fp.maxFeatures && nOctaveLevels==fp.nOctaveLevels && scaleFactor==fp.scaleFactor;}


        std::vector<float> getScaleFactors(){
            std::vector<float> res(nOctaveLevels);
            double sc=1;
            for(int i=0;i<nOctaveLevels;i++) {res[i]=sc;sc*=scaleFactor;}
            return res;
        }

        int getMaxFeatures( ){ return maxFeatures;}

    };

    enum F2S_Type{F2D_ORB=0,F2D_GRID_AKAZE=1,F2D_GRID_BRISK=2,F2D_GRID_ORB=3,F2D_GRID_FREAK=4,F2D_GRID_SURF=5};

    virtual  ~Feature2DSerializable(){}

    static std::shared_ptr<Feature2DSerializable> create(DescriptorTypes::Type type);

    void detectAndCompute( cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints,
                           cv::OutputArray descriptors, FeatParams params );

    void toStream( std::ostream &str);
    static std::shared_ptr<Feature2DSerializable> fromStream(std::istream &str);

    void setParams(const std::string &params){str_params=params;}

    virtual Feature2DSerializable::FeatParams getParams()const=0;

    virtual float getMinDescDistance()const=0;

    //sets a value to define how sensitive the descriptor is [0,1]. Values  near 0 should be used for well illuminates scenes.
    //Increase the value for dark scenes. Default value is 0
    virtual void setSensitivity(float v){}
    virtual float getSensitivity( ){return 0;}

    //a factor indicating the percentage of keypoints that must be repeated in another frame to consider it redundant
   virtual DescriptorTypes::Type getDescriptorType()const=0;

protected:
    virtual F2S_Type getType()const=0;
    virtual void detectAndCompute_impl( cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints,
                           cv::OutputArray descriptors, FeatParams params  )=0;
    virtual void toStream_impl(std::ostream &str)=0;
    virtual void fromStream_impl(std::istream &str)=0;
    std::string str_params;
};
}
#endif
