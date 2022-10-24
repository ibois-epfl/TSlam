
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
