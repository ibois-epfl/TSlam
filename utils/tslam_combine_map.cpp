#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <iostream>
#include <exception>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <type_traits>
#include "cvprojectpoint.h"
#include <tslam.h>
#include "utils/mapmanager.h"

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

namespace tslam {
struct DebugTest{
    static void removeMapPointObservation(tslam::Map &map,uint32_t point_idx,uint32_t frame_idx){
        map.removeMapPointObservation(point_idx,frame_idx,3);
    }
};
}

int main(int argc,char **argv){
    try {
        if(argc<4)throw std::runtime_error("Usage: inmap_A inmap_B outmap [iterations]");

        std::shared_ptr<tslam::Map> TheMapA, TheMapB;
        TheMapA = std::make_shared<tslam::Map>();
        TheMapB = std::make_shared<tslam::Map>();

        TheMapA->readFromFile(argv[1]);
        TheMapB->readFromFile(argv[2]);

        int niters=50;
        if(argc>=5)niters=stoi(argv[4]);

        TheMapA->merge(TheMapB);

        // TheMapB->projectTo(*TheMapA);

        // std::shared_ptr<tslam::MapManager> TheMapManager;
        // TheMapManager = std::make_shared<tslam::MapManager>();
        // TheMapManager->setParams(TheMapA, true);

        // std::map<uint32_t, tslam::Frame*> frameMapB; // idx of TheMapB -> TheMapA
        // std::map<uint32_t, tslam::MapPoint*> pointMapB;

        // // Add point first
        // cout << "Total points in A: " << TheMapA->map_points.size() << "points." << endl;
        // cout << "Total points in B: " << TheMapB->map_points.size() << "points." << endl;

        // for(auto ptIter = TheMapB->map_points.begin(); ptIter != TheMapB->map_points.end(); ++ptIter){
        //     if(pointMapB.count(ptIter->id) == 0){
        //         pointMapB[ptIter->id] = &TheMapA->addNewPoint(0);
        //         pointMapB[ptIter->id]->setCoordinates(ptIter->getCoordinates());
        //         pointMapB[ptIter->id]->setStable(true);
        //         pointMapB[ptIter->id]->setBad(false);
        //         pointMapB[ptIter->id]->setSeen();
        //         pointMapB[ptIter->id]->setVisible();
        //     }
        // }

        // for(auto kfIter = TheMapB->keyframes.begin(); kfIter != TheMapB->keyframes.end(); ++kfIter){
        //     for(int i = 0 ; i < kfIter->ids.size() ; i++){
        //         if (kfIter->ids[i] != std::numeric_limits<uint32_t>::max()){
        //             kfIter->ids[i] = pointMapB[kfIter->ids[i]]->id;
        //         }
        //     }
        //     TheMapManager->addKeyFrame(&(*kfIter));
        // }

        // cout << "Markers: ";
        // for(auto markerIter = TheMapA->map_markers.begin(); markerIter != TheMapA->map_markers.end(); ++markerIter){
        //     cout << markerIter->first << " ";
        // }
        // cout << endl;

        string filename = argv[3];
        TheMapA->saveToFile(filename+".no-opt");

        TheMapA->optimize(50);

        cout<<"Final Camera Params "<<endl;
        cout<<TheMapA->keyframes.begin()->imageParams.CameraMatrix<<endl;
        cout<<TheMapA->keyframes.begin()->imageParams.Distorsion<<endl;

        TheMapA->saveToFile(argv[3]);

    } catch (const std::exception &ex) {
        std::cerr<<ex.what()<<std::endl;
    }
}
