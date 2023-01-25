#include "tslam_reconstructor.hh"
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
        "/home/as/TSlam/src/reconstruction/tests/test_data/synth_parallel_XY_XZ_YZ_v_double_stripes.yml"        // 11  TODO:: this is not working, have a look
        };

    // (1) create timber object & read yml TSlam map
    std::shared_ptr<tslam::Reconstruction::TSTimber> timberPtr = 
        std::make_shared<tslam::Reconstruction::TSTimber>();
    timberPtr->setPlaneTagsFromYAML(testData[9]);

    // (2) create geometric solver
    std::shared_ptr<tslam::Reconstruction::TSGeometricSolver> solverPtr = 
        std::make_shared<tslam::Reconstruction::TSGeometricSolver>();

    // (2.1) set solver's parameters
    solverPtr->setTimber(timberPtr);
    solverPtr->setCreaseAngleThreshold(2.0);   ///< default: 10.0 deg (?)
    solverPtr->setMaxPlnDist2Merge(1.0);       ///< default: 1.0 deg
    solverPtr->setMaxPlnAngle2Merge(0.9);      ///< default: 0.9 --> 0.9 / 0.5 = 4.5 mm  FIXME: unsure of the function
    solverPtr->setAABBScaleFactor(1.5f);        ///< default: 2.0
    solverPtr->setMaxPolyTagDist(0.05);        ///< default: 0.05
    solverPtr->setEPS(1e-5);                   ///< default: 1e-5  TODO: in geo util funcs() expose and pass the tolerance

    // (2.2 for debug) set solver's visualizer parameters
    solverPtr->setShowVisualizer(true);
    solverPtr->setSolverVisualizerParams(/*drawTags*/               true,
                                         /*drawTagTypes*/           false,
                                         /*drawTagNormals*/         false,
                                         /*drawAabb*/               false,
                                         /*drawIntersectedPoly*/    true,
                                         /*drawSplittingSegments*/  true,
                                         /*drawSelectedFace*/       true,
                                         /*drawFinalMesh*/          true
                                         );
    
    // (2.3) I/O
    solverPtr->setDirOut("/home/as/TSlam/src/reconstruction/tests/test_data/");
    solverPtr->setFilenameOut("test_mesh_1.ply");

    // (2.4) run it!
    solverPtr->reconstruct();

    return 0;
}