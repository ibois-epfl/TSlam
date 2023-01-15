#include "ts_reconstruct.hh"
#include <iostream>
#include <fstream>
#include <sstream>


int main()
{
    // test data
    std::vector<std::string> testData = {
        "/home/as/TSlam/src/reconstruction/tests/test_data/long_map.yml",                        // 0
        "/home/as/TSlam/src/reconstruction/tests/test_data/mid_map.yml",                         // 1
        "/home/as/TSlam/src/reconstruction/tests/test_data/small_map.yml",                       // 2
        "/home/as/TSlam/src/reconstruction/tests/test_data/synth_big_xlarge_only_extreme.yml",   // 3
        "/home/as/TSlam/src/reconstruction/tests/test_data/synth_notch_xlong.yml",               // 4

        "/home/as/TSlam-gt-data/synth_yaml_out/#GTDATACOLL#0.yml",           // 3
        "/home/as/TSlam-gt-data/synth_yaml_out/#GTDATACOLL#1.yml",           // 4
        "/home/as/TSlam-gt-data/synth_yaml_out/#GTDATACOLL#2.yml",           // 5
        "/home/as/TSlam-gt-data/synth_yaml_out/#GTDATACOLL#3.yml",           // 6
        "/home/as/TSlam-gt-data/synth_yaml_out/#GTDATACOLL#4.yml",           // 7
        };

    // create timber object & read yml TSlam map
    std::shared_ptr<tslam::TSTimber> timberPtr = std::make_shared<tslam::TSTimber>();
    timberPtr->setPlaneTagsFromYAML(testData[4]);

    // create geometric solver
    std::shared_ptr<tslam::TSGeometricSolver> solverPtr = std::make_shared<tslam::TSGeometricSolver>();

    // set solver's parameters
    solverPtr->setTimber(timberPtr);
    solverPtr->setCreaseAngleThreshold(10.0);   ///< default: 10.0
    solverPtr->setMaxPlnDist2Merge(5.2);     ///< default: 5.2
    solverPtr->setMaxPlnAngle2Merge(0.9);
    solverPtr->setAABBScaleFactor(4.0);    ///< default: 2.0
    solverPtr->setMaxPolyTagDist(1e-5);    ///< default: 0.03

    // set solver's visualizer parameters
    solverPtr->setShowVisualizer(true);
    solverPtr->setSolverVisualizerParams(/*drawTags*/               true,
                                         /*drawTagTypes*/           true,
                                         /*drawTagNormals*/         false,
                                         /*drawAabb*/               true,
                                         /*drawSplittingSegments*/  true,
                                         /*drawSplitPoly*/          true,
                                         /*drawFinalMesh*/          false
                                         );
    
    // I/O
    solverPtr->setDirOut("/home/as/TSlam/src/reconstruction/tests/test_data/");
    solverPtr->setFilenameOut("test_mesh_1.ply");

    // run it!
    solverPtr->reconstruct();

    return 0;
}