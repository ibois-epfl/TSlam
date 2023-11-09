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
*/#ifndef tslam_ImageParmas_H
#define tslam_ImageParmas_H
#include <opencv2/core/core.hpp>
#include "tslam_exports.h"
namespace tslam{

    /**
 * @brief The ImageParams class represents the parameters of the employed camera
 */
    class  TSLAM_API ImageParams {

    public:

        // 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
        cv::Mat CameraMatrix;
        //  distortion matrix
        cv::Mat Distorsion;
        // size of the image
        cv::Size CamSize;

        std::vector<cv::Mat> arrayCamMatrix, arrayDistorsion, arrayRvec, arrayTvec;
        std::vector<cv::Size> multicams_cs;
        bool isArray()const{return arrayCamMatrix.size()!=0;}
        int arraySize()const{return arrayCamMatrix.size()+1;}



        float bl=0;//stereo camera base line
        float rgb_depthscale=1;//scale to obtain depth from the rgbd values
        int fisheye_model=0;//is a fisheye camera

        ImageParams();
        ImageParams(const ImageParams &IP);
        ImageParams& operator=(const ImageParams& ip) ;

        /**Reads from a YAML file generated with the opencv2.2 calibration utility
     */
        void readFromXMLFile(std::string filePath);
        void readArrayFromXMLFile(const std::string &filePath);
        void saveToXMLFile(std::string path );

        inline cv::Mat getCameraMatrix(int cam=0) const {
            if(cam==0) return CameraMatrix;
            else return arrayCamMatrix[cam-1];
        }

        inline cv::Mat getDistorsion(int cam=0) const{
            if(cam==0) return Distorsion;
            else return arrayDistorsion[cam-1];
        }

        //accessor to the main parameters of the camera model
        inline float fx(int cam=0)const{
            if(cam==0) return  CameraMatrix.ptr<float>(0)[0];
            else return arrayCamMatrix[cam-1].ptr<float>(0)[0];}
        inline float cx(int cam=0)const{
            if(cam==0) return  CameraMatrix.ptr<float>(0)[2];
            else return arrayCamMatrix[cam-1].ptr<float>(0)[2];}
        inline float fy(int cam=0)const{
            if(cam==0)return  CameraMatrix.ptr<float>(0)[4];
            else return arrayCamMatrix[cam-1].ptr<float>(0)[4];}
        inline float cy(int cam=0)const{if(cam==0)return  CameraMatrix.ptr<float>(0)[5];
            else return arrayCamMatrix[cam-1].ptr<float>(0)[5];}

        /**Indicates whether this object is valid
     */
        bool isValid() const;
        /**Adjust the parameters to the size of the image indicated
     */
        void resize(cv::Size size, int cam=0);
        /**OPerator ==
     */
        inline bool operator==(const ImageParams& ip)const{
            return getSignature()==ip.getSignature();
        }
        /**OPerator !=
     */
        inline bool operator!=(const ImageParams& ip)const{
            return getSignature()!=ip.getSignature();
        }

        inline bool isIntoImage(cv::Point2f &p2){
            if ( p2.x>=0 && p2.y>=0 && p2.x<CamSize.width &&  p2.y<CamSize.height) return true;
            return false;
        }

        void toStream(std::ostream &str) const ;
        void fromStream(std::istream &str) ;

        //apply distortion to a undistorted point
        //  std::vector<cv::Point2f> distortPoints(const std::vector<cv::Point2f> &p) const;

        //removes distortion of points_io. If pout!=0, results are set in pout. Otherwise, points_io is modified with the new points

        void undistortPoints(   std::vector<cv::Point2f> &points_io, std::vector<cv::Point2f> *pout=nullptr, int cam=0)const;


        //returns a version of this without distortion
        ImageParams undistorted()const{
            ImageParams ip=*this;
            ip.Distorsion.setTo(cv::Scalar::all(0));
            return ip;
        }

        void clear(){
            CameraMatrix=cv::Mat();
            Distorsion=cv::Mat();
            CamSize=cv::Size(-1,-1);
        }


        bool isStereoCamera()const{return bl!=0;}
        //indicates if a depth point is close or far
        inline bool isClosePoint(float z)const{
            return z<40*bl;
        }


        uint64_t getSignature()const;


        friend std::ostream &operator<<(std::ostream &str,const ImageParams &ip);


    private:

    };
}
#endif
