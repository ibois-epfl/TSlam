#include "tslam_reconstructor.hh"
#include <iostream>
#include <fstream>
#include <sstream>


int main()
{
    // (* for debug) test data
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
        "/home/as/TSlam/src/reconstruction/tests/test_data/synth_parallel_XY_XZ_YZ_v_double_stripes.yml",        // 11  TODO:: this is not working, have a look
        "/home/as/TSlam/src/reconstruction/tests/test_data/real_world_pyramid.yml"        // 12
        };

    // (0) create reconstructor
    tslam::Reconstruction::TSLAMReconstructor reconstructor = 
        tslam::Reconstruction::TSLAMReconstructor();
    
    // (0.1) load map
    reconstructor.loadMap(testData[4]);

    // (0.2) set parameters
    reconstructor.setParams(2.0,
                            1.0,
                            0.9,
                            1.5,
                            0.05,
                            1e-6);

    // (* for debug) set solver's visualizer parameters
    reconstructor.getGeometricSolver().setShowVisualizer(true);
    reconstructor.getGeometricSolver().setSolverVisualizerParams(/*drawTags*/               true,
                                                                 /*drawTagTypes*/           false,
                                                                 /*drawTagNormals*/         false,
                                                                 /*drawAabb*/               false,
                                                                 /*drawIntersectedPoly*/    false,
                                                                 /*drawSplittingSegments*/  true,
                                                                 /*drawSelectedFace*/       true,
                                                                 /*drawFinalMesh*/          false
                                                                 );

    // (2.4) run it!
    reconstructor.run();

    // (2.5) save mesh (as PLY or XAC)
    // reconstructor.saveMeshAsPLY("/home/as/TSlam/src/reconstruction/tests/test_data/", "test_mesh.ply");

    // (3) clean up (unload all vectors and delete all objects) if you want to run it again
    reconstructor.clean();

    // std::cout << "Done!" << std::endl;

    // reconstructor.loadMap(testData[4]);

    // std::cout << "Donex2!" << std::endl;

    // reconstructor.getGeometricSolver().setShowVisualizer(false);  // TEST


    // reconstructor.run();

    // std::cout << "Donex3!" << std::endl;



    return 0;
}