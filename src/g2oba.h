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
or implied, of Rafael Muñoz Salinas or Andrea Settimi and Hong-Bin Yang.
*/
#ifndef _g2oba_H_
#define _g2oba_H_

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "map_types/marker.h"
#include "cvprojectpoint.h"
namespace g2oba {


typedef Eigen::Matrix<double,9,1,Eigen::ColMajor>    Vector9D;
typedef Eigen::Matrix<double,6,1,Eigen::ColMajor>    Vector6D;


class   CameraParams : public g2o::BaseVertex<9, Vector9D>
{
    bool _isSet=false;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setParams(const cv::Mat &CameraMatrix,const cv::Mat &Dist  ){
        cv::Mat Cam64,Dist64;
        CameraMatrix.convertTo(Cam64,CV_64F);
        Dist.convertTo(Dist64,CV_64F);
        Eigen::Matrix<double,9,1> obs;
        obs(0)=Cam64.at<double>(0,0);//fx
        obs(1)=Cam64.at<double>(1,1);//fy
        obs(2)=Cam64.at<double>(0,2);//cx
        obs(3)=Cam64.at<double>(1,2);//cy
        for(int i=0;i<5;i++) obs(4+i)=Dist64.ptr<double>(0)[i];//distortion coeffs
        setEstimate(obs);
        _isSet=true;

    }
    bool isSet()const{return _isSet;}

    virtual void setToOriginImpl() {
        _estimate.fill(0);
    }
    virtual void oplusImpl(const number_t* update)
    {
        Eigen::Map<const Vector9D> v(update);
        _estimate += v;
    }
    inline double fx()const{return _estimate(0);}
    inline double fy()const{return _estimate(1);}
    inline double cx()const{return _estimate(2);}
    inline double cy()const{return _estimate(3);}
    inline const double * dist()const{return &_estimate(4);}

    bool read(std::istream& is){throw std::runtime_error("Not implemented");}
    bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}

};

/**
 * \brief SE3 Vertex parameterized internally with a transformation matrix
 and externally with its exponential map
 */
class   SE3Pose : public g2o::BaseVertex<6, Vector6D>{
    cv::Mat R,derR,RT;
    uint32_t _id;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    bool read(std::istream& is){throw std::runtime_error("Not implemented");}
    bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}

    virtual void setToOriginImpl() {
        _estimate.fill(0);
    }

    virtual void oplusImpl(const number_t* update)
    {
        Eigen::Map<const Vector6D> v(update);
        _estimate += v;
        prepare();
    }
    SE3Pose(uint32_t id, const cv::Mat &rvec,const cv::Mat &tvec):g2o::BaseVertex<6, Vector6D>(){
        _id=id;
        cv::Mat R64,T64;
        rvec.convertTo(R64,CV_64F);
        tvec.convertTo(T64,CV_64F);
        Eigen::Matrix<double,6,1> obs;
        for(int i=0;i<3;i++) obs(i)=R64.ptr<double>(0)[i];
        for(int i=0;i<3;i++) obs(3+i)=T64.ptr<double>(0)[i];
        setEstimate(obs);
        prepare();
    }
    uint32_t getid()const{return _id;}
    inline void prepare(){
        if(sizeof(_estimate(0))!=sizeof(double)) throw  std::runtime_error("MUST BE DOUBLE");
        cv::Mat rvec(1,3,CV_64F, &_estimate(0));
        cv::Rodrigues(rvec,R,derR);
        RT.create(4,4,CV_64F);
        R.copyTo(RT.rowRange(0,3).colRange(0,3));
        RT.at<double>(0,3)=_estimate(3);
        RT.at<double>(1,3)=_estimate(4);
        RT.at<double>(2,3)=_estimate(5);
        RT.at<double>(3,3)=1;
    }

    inline   double operator()(const int pos)const{return _estimate(pos);}
    inline const double * getRvec()const{return &_estimate(0);}
    inline const double * getTvec()const{return &_estimate(3);}

    const double *getRMat()const{return R.ptr<double>(0);}
    const double *getDervMat()const{return derR.ptr<double>(0);}

    cv::Mat getRT()const{return RT;}

};

/**
 * \brief Point vertex, XYZ
 */
class   MapPoint : public g2o::BaseVertex<3, g2o::Vector3>
{
    uint32_t _id=std::numeric_limits<uint32_t>::max();;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MapPoint(){}
    MapPoint(uint32_t id,const cv::Point3d &p):g2o::BaseVertex<3, g2o::Vector3>(){
        Eigen::Matrix<double,3,1> obs;
        obs<<p.x,p.y,p.z;
        _id=id;
        setEstimate(obs);
    }
    uint32_t getid()const{return _id;}
    virtual void oplusImpl(const number_t* update)
    {
        Eigen::Map<const g2o::Vector3> v(update);
        _estimate += v;
    }
    virtual void setToOriginImpl() {
        _estimate.fill(0);
    }
    cv::Point3d getPoint3d()const{
        return cv::Point3d(_estimate(0),_estimate(1),_estimate(2));
    }
    bool read(std::istream& is){throw std::runtime_error("Not implemented");}
    bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}

};


class  ProjectionEdge: public  g2o::BaseMultiEdge<2, g2o::Vector2>
{

    double dervs[2*18];//First row,  horizontal pixel error: rx,ry,rz,tx,ty,tx,fx,fy,cx,cy,k1,k2,p1,p2,k3,X,Y,Z Second row: same for vertical pixel error
public:
    uint32_t point_id,frame_id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    ProjectionEdge(uint32_t pointid,uint32_t frameid,cv::Point2d Proj){
        resize(3);
        point_id=pointid;
        frame_id=frameid;
    }

    inline void computeError()  {
        const  MapPoint* point= static_cast<const  g2oba::MapPoint *>(_vertices[0]);//
        const  CameraParams* cam= static_cast<const  g2oba::CameraParams*>(_vertices[1]);//
        const  SE3Pose* pos= static_cast<const  g2oba::SE3Pose*>(_vertices[2]);//

        auto proj=__projectPoint(point->getPoint3d(), pos->getRvec(),pos->getTvec(), cam->fx(),cam->fy(),cam->cx(),cam->cy(),cam->dist()
                                 ,dervs,pos->getRMat(),pos->getDervMat());
        _error.resize(2);
        _error(0)=proj.x-_measurement(0);
        _error(1)=proj.y-_measurement(1);
    }
    virtual void linearizeOplus(){
        {//Point Derivatives
            for(int r=0;r<2;r++){
                for(int c=0;c<3;c++){
                    _jacobianOplus[0](r,c)=dervs[r*18+15+c];
                }
            }

        }
        {//Camera derivatives
            for(int r=0;r<2;r++)
                for(int c=0;c<9;c++) _jacobianOplus[1](r,c)=dervs[ r*18+ 6+c];
        }
        {//Pose derivatives
            for(int r=0;r<2;r++)
                for(int c=0;c<6;c++) _jacobianOplus[2](r,c)=dervs[ r*18+ c];
        }
    }

    virtual bool read(std::istream& is){throw std::runtime_error("Not implemented");}
    virtual bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}
};



class  ControlPointEdge: public  g2o::BaseUnaryEdge<3, g2o::Vector3,g2oba::MapPoint>
{

 public:
    uint32_t point_id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ControlPointEdge(uint32_t pointid ){point_id=pointid;}

    inline void computeError()  {
        auto p=static_cast<const  g2oba::MapPoint *>(_vertices[0])->getPoint3d();
        _error.resize(3);
        _error(0)=p.x-_measurement(0);
        _error(1)=p.y-_measurement(1);
        _error(2)=p.z-_measurement(2);
    }
    virtual bool read(std::istream& is){throw std::runtime_error("Not implemented");}
    virtual bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}
};


class  ControlPointEdge2D: public  g2o::BaseUnaryEdge<2, g2o::Vector2,g2oba::MapPoint>
{

 public:
    uint32_t point_id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    ControlPointEdge2D(uint32_t pointid ){ point_id=pointid;}
    inline void computeError()  {
        auto p=static_cast<const  g2oba::MapPoint *>(_vertices[0])->getPoint3d();
        _error.resize(2);
        _error(0)=p.x-_measurement(0);
        _error(1)=p.y-_measurement(1);
    }
    virtual bool read(std::istream& is){throw std::runtime_error("Not implemented");}
    virtual bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}
};


typedef Eigen::Matrix<double,8,1,Eigen::ColMajor>    Vector8D;

class  MarkerEdge: public  g2o::BaseMultiEdge<8, Vector8D>
{
    std::vector<g2o::Vector3>  points;

    uint32_t marker_id,frame_id;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double fx, fy, cx, cy;

    MarkerEdge(const tslam::Marker &marker,uint32_t frameid){
        resize(3);
        marker_id=marker.id;
        frame_id=frameid;
        //  _delta_der=1e-4;//set the delta increment to compute the partial derivative for Jacobians
         for(auto p:marker.get3DPoints(false))
             points.push_back(  g2o::Vector3 ( p.x,p.y,p.z ));
    }

    bool read(std::istream& is){assert(false);return false;}

    bool write(std::ostream& os) const{assert(false);return false;}

    inline void computeError()  {
        //marker
        const  SE3Pose* g2m= static_cast<const  SE3Pose*>(_vertices[0]);//marker pose
        //camera
        const  SE3Pose*  c2g = static_cast<const  SE3Pose*>(_vertices[1]);//keyframe pose
        //camera params
        const CameraParams*cam=static_cast<const  CameraParams*>(_vertices[2]);


        //std::cout<<" ------------------------"<<std::endl;
        auto Transform_C2M=c2g->getRT()*  g2m->getRT();
        //now, project
        _error.resize(8);
        for(int i=0;i<4;i++){
            auto p3d= apply(  points[i],Transform_C2M);//3d rigid transform
            auto proj= __projectPoint( cv::Point3d(p3d(0),p3d(1),p3d(2)),nullptr,nullptr,cam->fx(),cam->fy(),cam->cx(),cam->cy(),cam->dist());
            _error(i*2)=_measurement(i*2)-proj.x;
            _error(i*2+1)=_measurement(i*2+1)-proj.y;
            // if( std::isnan(_error(i*2)) || std::isnan(_error(i*2+1))){
            //     std::cout<<c2g->getRT()<<std::endl;
            //     std::cout<<g2m->getRT()<<std::endl;
            //     std::cout<<Transform_C2M<<std::endl;

            //     std::cerr<<"JERE"<<std::endl;
            // }
        }
    }

private:


   inline  g2o::Vector3 apply(const g2o::Vector3 &p,const cv::Mat & RT){

        const double *rt=RT.ptr<double>(0);
        g2o::Vector3 res;
        res(0)=rt[0]*p(0)+rt[1]*p(1)+rt[2]*p(2)+rt[3];
        res(1)=rt[4]*p(0)+rt[5]*p(1)+rt[6]*p(2)+rt[7];
        res(2)=rt[8]*p(0)+rt[9]*p(1)+rt[10]*p(2)+rt[11];
        return res;
    }
};

}
#endif

