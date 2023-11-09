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

#include "feature2dserializable.h"
#include "basictypes/io_utils.h"
#include "ORBextractor.h"
#if  CV_MAJOR_VERSION >= 3
#include "gridextractor.h"
#endif
namespace tslam{

void Feature2DSerializable::detectAndCompute(cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints,
                                              cv::OutputArray descriptors, FeatParams params  ){

    detectAndCompute_impl(image,mask,keypoints,descriptors,params);
}

std::shared_ptr<Feature2DSerializable> Feature2DSerializable::create(DescriptorTypes::Type type ){
    std::shared_ptr<Feature2DSerializable> fdetector;

    if(type==DescriptorTypes::DESC_ORB) {
            auto orbmur= std::make_shared<tslam::ORBextractor>( );
            fdetector=orbmur;
    }
#if  CV_MAJOR_VERSION >= 3

    else  if (type==DescriptorTypes::DESC_AKAZE){
        //          _params.keyFrameCullingPercentage=0.90;
        fdetector=std::make_shared<GridExtractor>();
        auto gextractor=std::make_shared<GridExtractor>();
        gextractor->setParams(GridExtractor::F2D_GRID_AKAZE );
        fdetector=gextractor;
    }
    else  if (type==DescriptorTypes::DESC_BRISK){
        //          _params.keyFrameCullingPercentage=0.75;
        auto gextractor=std::make_shared<GridExtractor>();
        gextractor->setParams(GridExtractor::F2D_GRID_BRISK );
        fdetector=gextractor;
    }


    else if (type==DescriptorTypes::DESC_FREAK){
        //          _params.keyFrameCullingPercentage=0.75;
        auto gextractor=std::make_shared<GridExtractor>();
        gextractor->setParams(GridExtractor::F2D_GRID_FREAK );
        fdetector=gextractor;
    }
    else if (type==DescriptorTypes::DESC_SURF){
        //          _params.keyFrameCullingPercentage=0.75;
        auto gextractor=std::make_shared<GridExtractor>();
        gextractor->setParams(GridExtractor::F2D_GRID_SURF );
        fdetector=gextractor;
    }

#endif
    else throw std::runtime_error("Invalid input descriptor");
    return fdetector;
}


void Feature2DSerializable::toStream(std::ostream &str)
{
    uint64_t sig=1828374733;
    str.write((char*)&sig,sizeof(sig));
    //write type first
    uint64_t t=(uint64_t)getType();
    str.write((char*)&t,sizeof(t));
    toStream__(str_params,str);
    toStream_impl(str);
}

std::shared_ptr<Feature2DSerializable> Feature2DSerializable::fromStream(std::istream &str){
    //
    std::shared_ptr<Feature2DSerializable> ptr;
    uint64_t sig;
    str.read((char*)&sig,sizeof(sig));
    if (sig!=1828374733)
        throw std::runtime_error("Feature2DSerializable::fromStream signature error in stream");
    uint64_t t;
    str.read((char*)&t,sizeof(t));
    switch(t){
    case F2D_ORB:
        ptr=std::make_shared<ORBextractor>();
        break;
#if  CV_MAJOR_VERSION >= 3
    case F2D_GRID_AKAZE:
    case F2D_GRID_BRISK:
    case F2D_GRID_ORB:
    case F2D_GRID_FREAK:
    case F2D_GRID_SURF:
        ptr=std::make_shared<GridExtractor>();
        break;
#endif
    };
    fromStream__(ptr->str_params,str);

    ptr->fromStream_impl(str);
    return ptr;
}
}
