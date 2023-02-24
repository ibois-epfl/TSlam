#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "tslam_reconstructor.hh"

#include <iostream>


/// File path
std::string ymlPath;

/// Dirs yml files
const std::string YML_DIR_REAL = "/home/as/TSlam/src/reconstruction/tests/test_data/real_scans/";
const std::string YML_DIR_SYNTH = "/home/as/TSlam/src/reconstruction/tests/test_data/synthetic_scans/";

/// GT data
uint GT_nbrTags;
uint GT_nbrPlnAABBPolygons;
uint GT_nbrSplitSegments;
uint GT_nbrFacePolygons_UP;
uint GT_nbrFacePolygons_DOWN;
uint GT_nbrMeshVertices_UP;
uint GT_nbrMeshVertices_DOWN;

/// Reconstructor
tslam::Reconstruction::TSLAMReconstructor reconstructor = 
        tslam::Reconstruction::TSLAMReconstructor();

/// Main check func
void checkResults(std::string ymlPath,
                  bool showVisualizer = false,
                  bool saveMesh = false)
{
    // load map for construction
    reconstructor.loadMap(ymlPath);

    // set params for visualizer
    reconstructor.getGeometricSolver().setShowVisualizer(showVisualizer);  ///< Enable/disable visualizer
    reconstructor.getGeometricSolver().setSolverVisualizerParams(/*drawTags*/               true,
                                                                 /*drawTagNormals*/         false,
                                                                 /*drawAabb*/               true,
                                                                 /*drawIntersectedPoly*/    false,
                                                                 /*drawSplittingSegments*/  false,
                                                                 /*drawSelectedFace*/       true,
                                                                 /*drawFinalMesh*/          true
    );

    // reconstruct!
    reconstructor.run();

    // check the inner values of the geometric solver
    auto internGeoSolver = reconstructor.getGeometricSolver();

    CHECK(internGeoSolver.checkMeshSanity() == true);

    reconstructor.saveMeshAsPLY(".", "test2_mesh");

    // clean everything
    reconstructor.clean();
}

// #############################################################################################
// TESTING
// #############################################################################################


TEST_SUITE("Real_scans")
{
    TEST_CASE("Long_length"
    * doctest::description("Checks the case of a streight beam with long length."))
    {
        ymlPath = YML_DIR_REAL + "long_map.yml";
        checkResults(ymlPath, true);
    }

    TEST_CASE("Medium_length"
    * doctest::description("Checks the case of a streight beam with medium length."))
    {
        ymlPath = YML_DIR_REAL + "mid_map.yml";
        checkResults(ymlPath, true);
    }

    // ===============================================================================

    TEST_CASE("Long_length_only_extremes_good_mapping"
    * doctest::description("Checks the case of a beam with tags only at extremeties."))
    {
        ymlPath = YML_DIR_REAL + "only_end_enhanced_2_TEST.yml";
        checkResults(ymlPath, true);
    }

    // // ===============================================================================

    TEST_CASE("Long_length_2_lap_joint"
    * doctest::description("Checks the case of a long beam with 2 lap joint."))
    {
        ymlPath = YML_DIR_REAL + "long_cut.yml";
        checkResults(ymlPath, true);
    }

    TEST_CASE("Medium_length_2_lap_joint"
    * doctest::description("Checks the case of a medium beam with 2 lap joint."))
    {
        ymlPath = YML_DIR_REAL + "medium_cut.yml";
        checkResults(ymlPath, true);
    }

    // ===============================================================================

    TEST_CASE("Complex_geometry_romboid"
    * doctest::description("Checks the case of a romboid object with multiple details."))
    {
        ymlPath = YML_DIR_REAL + "romboid.yml";
        checkResults(ymlPath, true);
    }
}

// ####################################################################################
// ####################################################################################

TEST_SUITE("Synthetic_scans")
{
    TEST_CASE("Long_length"
    * doctest::description("Checks the case of a streight beam with long length."))
    {
        ymlPath = YML_DIR_SYNTH + "synth_long_no_ruptures.yml";
        checkResults(ymlPath);
    }

    TEST_CASE("Medium_length"
    * doctest::description("Checks the case of a streight beam with medium length."))
    {
        ymlPath = YML_DIR_SYNTH + "synth_copy_mid_map.yml";
        checkResults(ymlPath);
    }

    // ===============================================================================

    TEST_CASE("Long_length_only_extremes"
    * doctest::description("Checks the case of a beam with tags only at extremeties."))
    {
        ymlPath = YML_DIR_SYNTH + "synth_big_xlarge_only_extreme.yml";
        checkResults(ymlPath);
    }

    // ===============================================================================

    TEST_CASE("Long_length_diagonal_lap_joint"
    * doctest::description("Checks the case of a beam with one lap joint."))
    {
        ymlPath = YML_DIR_SYNTH + "synth_notch_xlong.yml";
        checkResults(ymlPath);
    }

    // ===============================================================================

    TEST_CASE("Long_length_parallel_horizontal_XY_XZ_YZ_center_in_origin"
    * doctest::description("Checks the case of horizontal beam parallel to XY, XZ, YZ planes.\n"
                           "The beam's center is in the origin of world axis."))
    {
        ymlPath = YML_DIR_SYNTH + "synth_parallel_XY_XZ_YZ_h.yml";
        checkResults(ymlPath);
    }

    TEST_CASE("Long_length_parallel_vertical_XY_XZ_YZ_center_in_origin"
    * doctest::description("Checks the case of vertical beam parallel to XY, XZ, YZ planes.\n"
                           "The beam's center is in the origin of world axis."))
    {
        ymlPath = YML_DIR_SYNTH + "synth_parallel_XY_XZ_YZ_v.yml";
        checkResults(ymlPath);
    }

    TEST_CASE("Long_length_parallel_vertical_XY_XZ_YZ"
    * doctest::description("Checks the case of vertical beam parallel to XY, XZ, YZ planes."))
    {
        ymlPath = YML_DIR_SYNTH + "synth_parallel_XY_XZ_YZ_v_not_symmetrical.yml";
        checkResults(ymlPath);
    }

    TEST_CASE("Long_length_parallel_vertical_XY_XZ_YZ_double_stripes"
    * doctest::description("Checks the case of vertical beam parallel to XY, XZ, YZ planes.\n"
                           "Stripes are doubled on each face."))
    {
        ymlPath = YML_DIR_SYNTH + "synth_parallel_XY_XZ_YZ_v_double_stripes.yml";
        checkResults(ymlPath);
    }
}

