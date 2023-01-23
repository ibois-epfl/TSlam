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
        "/home/as/TSlam/src/reconstruction/tests/test_data/synth_copy_mid_map.yml",              // 5
        "/home/as/TSlam/src/reconstruction/tests/test_data/synth_long_no_ruptures.yml",          // 6
        "/home/as/TSlam/src/reconstruction/tests/test_data/synth_parallel_XY_XZ_YZ_h.yml",       // 7
        "/home/as/TSlam/src/reconstruction/tests/test_data/synth_parallel_XY_XZ_YZ_v.yml",       // 8
        "/home/as/TSlam/src/reconstruction/tests/test_data/synth_parallel_XY_XZ_YZ_v_not_symmetrical.yml",        // 9
        "/home/as/TSlam/src/reconstruction/tests/test_data/synth_parallel_XY_XZ_YZ_h_rot.yml",        // 10
        "/home/as/TSlam/src/reconstruction/tests/test_data/synth_parallel_XY_XZ_YZ_v_double_stripes.yml"        // 11
        };

    // create timber object & read yml TSlam map
    std::shared_ptr<tslam::TSTimber> timberPtr = std::make_shared<tslam::TSTimber>();
    timberPtr->setPlaneTagsFromYAML(testData[4]);  // 4

    // create geometric solver
    std::shared_ptr<tslam::TSGeometricSolver> solverPtr = std::make_shared<tslam::TSGeometricSolver>();

    // set solver's parameters
    solverPtr->setTimber(timberPtr);
    solverPtr->setCreaseAngleThreshold(5.0);   ///< default: 10.0 deg
    solverPtr->setMaxPlnDist2Merge(1.0);       ///< default: 1.0 deg
    solverPtr->setMaxPlnAngle2Merge(0.9);      ///< default: 0.9 --> 0.9 / 0.5 = 4.5 mm
    solverPtr->setAABBScaleFactor(1.5f);        ///< default: 2.0
    solverPtr->setMaxPolyTagDist(1e-5);        ///< default: 1e-5
    solverPtr->setEPS(1e-5);                   ///< default: 1e-5  TODO: in geo util funcs() expose and pass the tolerance

    // set solver's visualizer parameters
    solverPtr->setShowVisualizer(true);
    solverPtr->setSolverVisualizerParams(/*drawTags*/               true,
                                         /*drawTagTypes*/           true,
                                         /*drawTagNormals*/         false,
                                         /*drawAabb*/               false,
                                         /*drawIntersectedPoly*/    false,
                                         /*drawSplittingSegments*/  true,
                                         /*drawSelectedFace*/       true,
                                         /*drawFinalMesh*/          false
                                         );
    
    // I/O
    solverPtr->setDirOut("/home/as/TSlam/src/reconstruction/tests/test_data/");
    solverPtr->setFilenameOut("test_mesh_1.ply");

    // run it!
    solverPtr->reconstruct();

    return 0;
}