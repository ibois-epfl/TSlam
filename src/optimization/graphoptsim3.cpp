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
#include "graphoptsim3.h"
#include "typesg2o.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "map_types/covisgraph.h"
#include "basictypes/debug.h"
#include <Eigen/Geometry>
#include <iostream>
using namespace std;
namespace tslam{
namespace gop_sim3 {

g2o::Sim3 getSim3 (const cv::Mat &T,float s=1){
    auto toMatrix3d=[](const cv::Mat &cvMat3)
    {
        Eigen::Matrix<double,3,3> M;

        M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
             cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
             cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

        return M;
    };
     auto  toVector3d=[](const cv::Mat &cvVector)
    {
        Eigen::Matrix<double,3,1> v;
        v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

        return v;
    };

        return g2o::Sim3 (toMatrix3d(T.rowRange(0,3).colRange(0,3)),toVector3d(T.rowRange(0,3).colRange(3,4)),s);
};

cv::Mat  toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }

    return cvMat;
}
}

using namespace gop_sim3;
void    loopClosurePathOptimizationg2o(const std::vector<std::pair<uint32_t,uint32_t> > &edges, uint32_t IdClosesLoopNew,
                                       uint32_t IdClosesLoopOld, cv::Mat expectedPoseNew, std::map<uint32_t, cv::Mat> &optimPoses,
                                       bool bFixScale,std::map<uint64_t,float> edgeWeight){

    return;
    auto isClosingLoopEdge=[&](const std::pair<uint32_t,uint32_t> &edge){
         return( (edge.first==IdClosesLoopNew && edge.second==IdClosesLoopOld) || (edge.second==IdClosesLoopNew && edge.first==IdClosesLoopOld));
    };

    //Create optimizer
    g2o::SparseOptimizer Optimizer;
    std::unique_ptr<g2o::BlockSolver_7_3::LinearSolverType> linearSolver=g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_7_3>(std::move(linearSolver)));
    Optimizer.setAlgorithm(solver);
    solver->setUserLambdaInit(1e-16);
    Optimizer.setAlgorithm(solver);
    std::map<uint32_t,VertexSim3Expmap*> vertices;

//cout<<"ADDING VERTICES"<<endl;
    //add vertices
    for(const auto &EdgePose:optimPoses){

        VertexSim3Expmap* VSim3 = new VertexSim3Expmap();

        if(EdgePose.first==IdClosesLoopNew){
            VSim3->setEstimate(getSim3(expectedPoseNew,1));
        }
        else{
            VSim3->setEstimate(getSim3(EdgePose.second,1));
        }

        VSim3->setFixed(EdgePose.first== IdClosesLoopOld );
        VSim3->setId(EdgePose.first);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;
        Optimizer.addVertex(VSim3);
        vertices[EdgePose.first]=VSim3;
    }
   // cout<<"ADDING EDGES"<<endl;


    // Add  edges
    Eigen::Matrix<double,7,7> matLambda= Eigen::Matrix<double,7,7>::Identity();
     for(const auto &edge:edges){

        g2o::Sim3 Siw,Sjw;
        assert(optimPoses.count(edge.first)!=0);
        assert(optimPoses.count(edge.second)!=0);
        if ( isClosingLoopEdge(edge)){
            Siw=(edge.first == IdClosesLoopNew)? getSim3(expectedPoseNew):getSim3(optimPoses.at(edge.first));
            Sjw=(edge.second== IdClosesLoopNew)? getSim3(expectedPoseNew):getSim3(optimPoses.at(edge.second));
        }
        else{
            Siw=getSim3(optimPoses.at(edge.first));
            Sjw=getSim3(optimPoses.at(edge.second));
        }
        const g2o::Sim3 Swi = Siw.inverse();
        const g2o::Sim3 Sji = Sjw * Swi;
        EdgeSim3* e = new EdgeSim3();
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(Optimizer.vertex(edge.first)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(Optimizer.vertex(edge.second)));
        e->setMeasurement(Sji);

        //set information as the weight of the edge
        auto edw=edgeWeight.find(CovisGraph::join(edge.first,edge.second));
        float weight=1;
        if (edw!=edgeWeight.end())
            weight=edw->second;

         e->information() = weight*matLambda;

        Optimizer.addEdge(e);
    }

    // Optimize!
    //cout<<"INITIALIZING"<<endl;
    if(!Optimizer.initializeOptimization()){
        return; // this one I'm not sure if the rest stuff need to be run (the for loop of vertices)
    }
    Optimizer.setVerbose(debug::Debug::getLevel()>=10);
   // cout<<"OPTIMIZING"<<endl;
    Optimizer.optimize(20);


    for(auto v:vertices){
         g2o::Sim3 CorrectedSiw =v.second->estimate();
         Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
         Eigen::Vector3d eigt = CorrectedSiw.translation();
         double s = CorrectedSiw.scale();
         eigt *=(1./s);
         eigR *=s;
         //[sR t;0 1]
         optimPoses.at(v.first)=toCvSE3(eigR,eigt);
    }
}

}
