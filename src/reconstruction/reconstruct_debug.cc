#include "ts_reconstruct.hh"
#include <iostream>
#include <fstream>
#include <sstream>


int main()
{
    // test data
    std::vector<std::string> testData = {
        "/home/as/TSlam/src/reconstruction/tests/test_data/long_map.yml",
        "/home/as/TSlam/src/reconstruction/tests/test_data/mid_map.yml",
        "/home/as/TSlam/src/reconstruction/tests/test_data/small_map.yml"};

    // create timber object & read yml TSlam map
    std::shared_ptr<tslam::TSTimber> timberPtr = std::make_shared<tslam::TSTimber>();
    timberPtr->setPlaneTagsFromYAML(testData[0]);

    // create geometric solver
    std::shared_ptr<tslam::TSGeometricSolver> solverPtr = std::make_shared<tslam::TSGeometricSolver>();

    // set solver's parameters
    solverPtr->setTimber(timberPtr);
    solverPtr->setCreaseAngleThreshold(10.0);   ///< default: 10.0
    solverPtr->setMaxPlnDist2Merge(5.2);
    solverPtr->setMaxPlnAngle2Merge(0.9);
    solverPtr->setAABBScaleFactor(2.0);
    solverPtr->setMaxPolyTagDist(1e-5);    ///< default: 0.03

    // set solver's visualizer parameters
    solverPtr->setShowVisualizer(true);
    solverPtr->setSolverVisualizerParams(/*drawTags*/               true,
                                         /*drawTagTypes*/           true,
                                         /*drawTagNormals*/         true,
                                         /*drawAabb*/               true,
                                         /*drawSplittingSegments*/  true,
                                         /*drawSplitPoly*/          true,
                                         /*drawFinalMesh*/          false
                                         );

    // run it!
    solverPtr->reconstruct();

    return 0;
}