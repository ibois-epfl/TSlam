#include "ts_rtag.hh"
#include "ts_timber.hh"
#include "ts_geometric_solver.hh"

#include <iostream>
#include <fstream>
#include <sstream>

#include <algorithm>



int main()
{
    // create timber object & read yml TSlam map
    std::shared_ptr<tslam::TSTimber> timberPtr = std::make_shared<tslam::TSTimber>();
    timberPtr->setPlaneTagsFromYAML("/home/as/TSlam/src/reconstruction/long_comb.yml");

    // create geometric solver
    std::shared_ptr<tslam::TSGeometricSolver> solverPtr = std::make_shared<tslam::TSGeometricSolver>();

    // set solver's parameters
    solverPtr->setTimber(timberPtr);
    solverPtr->setCreaseAngleThreshold(10.0);
    solverPtr->setMinPolyDist(3.0);

    // solve
    solverPtr->reconstruct();


    return 0;
}