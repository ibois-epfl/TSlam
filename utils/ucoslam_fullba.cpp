#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include <iostream>
#include <exception>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <type_traits>
#include "cvprojectpoint.h"
#include "ucoslam.h"
#include "g2oba.h"
class CmdLineParser{int argc; char **argv;
                public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                    std::vector<std::string> getAllInstances(string str){
                        std::vector<std::string> ret;
                        for(int i=0;i<argc-1;i++){
                            if (string(argv[i])==str)
                                ret.push_back(argv[i+1]);
                        }
                        return ret;
                    }
                   };

namespace ucoslam {
struct DebugTest{
    static void removeMapPointObservation(ucoslam::Map &map,uint32_t point_idx,uint32_t frame_idx){
        map.removeMapPointObservation(point_idx,frame_idx,3);
    }
};
}
int main(int argc,char **argv){

    try {
        if(argc<3)throw std::runtime_error("Usage: inmap outmap [iterations]");

        ucoslam::Map TheMap;
        cout<<"reading map"<<endl;
        TheMap.readFromFile(argv[1]);
        cout<<"Done"<<endl;
        int niters=100;
        if(argc>=4)niters=stoi(argv[3]);

        std::shared_ptr<g2o::SparseOptimizer> Optimizer;

        Optimizer=std::make_shared<g2o::SparseOptimizer>();
        std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver=g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));


        Optimizer->setAlgorithm(solver);

        int id=0;
        //first, the cameras
        g2oba::CameraParams * camera = new g2oba::CameraParams();
        camera->setId(id++);



        //////////////////////////////
        ///KEYFRAME POSES
        //////////////////////////////
        map<uint32_t, g2oba::SE3Pose * > kf_poses;
        for(auto &kf:TheMap.keyframes){
            if(!camera->isSet()){
                camera->setParams(kf.imageParams.CameraMatrix,kf.imageParams.Distorsion);
                Optimizer->addVertex(camera);
            }
            g2oba::SE3Pose * pose= new g2oba::SE3Pose(kf.idx, kf.pose_f2g.getRvec(),kf.pose_f2g.getTvec());
            pose->setId(id++);
            Optimizer->addVertex(pose);
            kf_poses.insert({kf.idx,pose});
        }
        //////////////////////////////
        /// MAP POINTS
        //////////////////////////////
        list<g2oba::ProjectionEdge *> projectionsInGraph;
        list<g2oba::MapPoint *> mapPoints;
        for(auto &p:TheMap.map_points){
            g2oba::MapPoint *point=new g2oba::MapPoint (p.id, p.getCoordinates());
            point->setId(id++);
            point->setMarginalized(true);
            Optimizer->addVertex(point);
            mapPoints.push_back(point);

            for(auto of:p.getObservingFrames()){
                auto &kf=TheMap.keyframes[of.first];
                if ( kf.isBad() )continue;

                auto Proj=new g2oba::ProjectionEdge(p.id,kf.idx, kf.kpts[of.second]);
                Proj->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(point));
                Proj->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( camera));
                Proj->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>( kf_poses.at(kf.idx)));
                Eigen::Matrix<double,2,1> obs;
                obs<<kf.kpts[of.second].x,kf.kpts[of.second].y;
                Proj->setMeasurement(obs);
                Proj->setInformation(Eigen::Matrix2d::Identity()* 1./ kf.scaleFactors[kf.und_kpts[of.second].octave]);
                g2o::RobustKernelHuber* rk = new  g2o::RobustKernelHuber();
                rk->setDelta(sqrt(5.99));
                Proj->setRobustKernel(rk);
                Optimizer->addEdge(Proj);
                projectionsInGraph.push_back(Proj);
            }
        }

        //////////////////////////////
        ////MARKER POSES
        //////////////////////////////
        map<uint32_t, g2oba::SE3Pose * > marker_poses;
        std::vector<g2oba::MarkerEdge * > marker_edges;
        for(const auto &marker:TheMap.map_markers){
            if(!marker.second.pose_g2m.isValid()) continue;
            g2oba::SE3Pose * marker_pose= new g2oba::SE3Pose(marker.first,marker.second.pose_g2m.getRvec(),marker.second.pose_g2m.getTvec());
            marker_pose->setId(id++);
            Optimizer->addVertex(marker_pose);
            marker_poses.insert({marker.first,marker_pose});

            for(const auto &kfidx:marker.second.frames){
                auto &kf=TheMap.keyframes[kfidx];
                if ( kf.isBad() )continue;
                if(  kf_poses.count(kfidx)==0)throw std::runtime_error("Key frame for marker not in the optimization:"+std::to_string(kfidx));

                auto Proj=new g2oba::MarkerEdge(marker.second,kfidx);
                Proj->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(marker_pose));
                Proj->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( kf_poses.at(kfidx)));
                Proj->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>( camera));
                auto mobs=TheMap.keyframes[kfidx].getMarker(marker.first);
                Eigen::Matrix<double,8,1> obs;
                obs<<mobs.corners[0].x,mobs.corners[0].y,
                        mobs.corners[1].x,mobs.corners[1].y,
                        mobs.corners[2].x,mobs.corners[2].y,
                        mobs.corners[3].x,mobs.corners[3].y;

                Proj->setMeasurement(obs);
                Proj->setInformation(Eigen::Matrix< double, 8, 8 >::Identity());
                g2o::RobustKernelHuber* rk = new  g2o::RobustKernelHuber();
                rk->setDelta(sqrt(15.507));
                Proj->setRobustKernel(rk);
                Optimizer->addEdge(Proj);
                marker_edges.push_back(Proj);
            }
        }





        //////////////////////////////
        /// OPTIMIZE
        //////////////////////////////


        Optimizer->initializeOptimization();
        //    Optimizer->setForceStopFlag( );
        Optimizer->setVerbose(true);
        Optimizer->optimize(niters,1e-5);
        //now remove outliers
        for(auto &p:projectionsInGraph)
            p->setLevel(p->chi2()>5.99);
        for(auto &p:marker_edges)
            p->setLevel(p->chi2()>15.507);

        Optimizer->optimize(niters,1e-4);

        //copy data back to the map
        //move data back to the map
        for(auto &mp: mapPoints ){
            TheMap.map_points[ mp->getid()].setCoordinates(mp->getPoint3d());
        }

        //now, the keyframes
        for(auto pose:kf_poses){
            TheMap.keyframes[pose.first].pose_f2g=ucoslam::se3((*pose.second)(0),(*pose.second)(1),(*pose.second)(2),(*pose.second)(3),(*pose.second)(4),(*pose.second)(5));
            TheMap.keyframes[pose.first].imageParams.CameraMatrix.at<float>(0,0)=camera->fx();
            TheMap.keyframes[pose.first].imageParams.CameraMatrix.at<float>(1,1)=camera->fy();
            TheMap.keyframes[pose.first].imageParams.CameraMatrix.at<float>(0,2)=camera->cx();
            TheMap.keyframes[pose.first].imageParams.CameraMatrix.at<float>(1,2)=camera->cy();
            for(int p=0;p<5;p++)
                TheMap.keyframes[pose.first].imageParams.Distorsion.ptr<float>(0)[p]=camera->dist()[p];
        }
//        //remove weak links
//        for(auto &p:projectionsInGraph){
//            if(p->chi2()>5.99) ucoslam::DebugTest::removeMapPointObservation(TheMap,p->point_id,p->frame_id);
//        }

        //finally, markers
        for(auto pose:marker_poses)
            TheMap.map_markers[pose.first].pose_g2m=ucoslam::se3((*pose.second)(0),(*pose.second)(1),(*pose.second)(2),(*pose.second)(3),(*pose.second)(4),(*pose.second)(5));


        cout<<"Final Camera Params "<<endl;
        cout<<TheMap.keyframes.begin()->imageParams.CameraMatrix<<endl;
        cout<<TheMap.keyframes.begin()->imageParams.Distorsion<<endl;

        TheMap.saveToFile(argv[2]);

    } catch (const std::exception &ex) {
        std::cerr<<ex.what()<<std::endl;
    }
}
