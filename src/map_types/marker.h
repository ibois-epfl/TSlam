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
#ifndef tslam_Marker_H
#define tslam_Marker_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <set>
#include "tslam_exports.h"
#include "basictypes/se3.h"
#include "basictypes/se3transform.h"
namespace tslam {

/**A marker in the 3D space.
 */

class TSLAM_API Marker{
public:
    Marker(){}
    Marker(uint32_t Id,Se3Transform g2m):id(Id),pose_g2m(g2m){}

    std::vector<cv::Point3f> points3d;//three dimentional points of the marker wrt its center

    uint32_t id;//id
    Se3Transform pose_g2m=Se3Transform(true);//pose  Marker -> Global
    std::set<uint32_t> frames;//key frames in which the marker is visible
    std::string dict_info; //information about the dictionary it belongs to

    //returns the 3d points of the marker in the global_ref
    std::vector<cv::Point3f> get3DPoints(bool global_ref=true)const;
//    static std::vector<cv::Point3f> get3DPoints(Se3Transform pose_g2m, float size, bool global_ref=true);
    //returns the 3d points in the local reference system
//    static std::vector<cv::Point3f> get3DPointsLocalRefSystem( float size );
    //---------------------
    //serialization routines
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str) ;
    uint64_t getSignature(bool print=false)const;

};

//define the set of poses returned by the IPPE algorithm for a given marker
struct MarkerPosesIPPE
{
    cv::Mat sols[2];
    double errs[2];
    double err_ratio;
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);

    MarkerPosesIPPE(){}
    MarkerPosesIPPE(const MarkerPosesIPPE&M)
    {
        M.copyTo(*this);
    }
    MarkerPosesIPPE & operator=(const MarkerPosesIPPE&M){
        M.copyTo(*this);
        return *this;
    }

    void copyTo(MarkerPosesIPPE &mposes)const{
        for(int i=0;i<2;i++){
            sols[i].copyTo(mposes.sols[i]);
            mposes.errs[i]=errs[i];
        }
        mposes.err_ratio=err_ratio;
    }
    uint64_t getSignature()const;

};
/**The projection of a marker in an image
 */
class TSLAM_API MarkerObservation:public Marker{
public:
    MarkerObservation(){}
    MarkerObservation(const Marker &m):Marker(m){}
    std::vector<cv::Point2f> corners;//original corners in the image
    std::vector<cv::Point2f> und_corners;//undistored corners
    MarkerPosesIPPE poses;

    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str) ;
    uint64_t getSignature()const;
    void draw(cv::Mat& in, cv::Scalar color=cv::Scalar(0,0,255), int lineWidth = -1, bool writeId = true,bool writeInfo=false) const;

};

};
#endif
