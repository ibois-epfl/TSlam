#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "tslam_reconstructor.hh"

#include <iostream>



int fact(int n)
{
    return n <= 1 ? n : fact(n - 1) * n;
}

TEST_CASE("testing the factorial function")
{
    tslam::Reconstruction::TSLAMReconstructor reconstructor = 
        tslam::Reconstruction::TSLAMReconstructor();
    
    // (0.1) load map
    reconstructor.loadMap("/home/as/TSlam/src/reconstruction/tests/test_data/synth_notch_xlong.yml");

    // (0.2) set parameters
    reconstructor.setParams(2.0,
                            1.0,
                            0.9,
                            1.5,
                            0.05,
                            1e-6);
    
    // (* for debug) set solver's visualizer parameters
    reconstructor.getGeometricSolver().setShowVisualizer(false);

    // (2.4) run it!
    reconstructor.run();

    // CHECK(fact(0) == 1); // should fail
    CHECK(fact(1) == 1);
    CHECK(fact(2) == 2);
    // CHECK(fact(3) == 3);
    // CHECK(fact(10) == 3628800);
}

/*
    For all the yml map files we

*/