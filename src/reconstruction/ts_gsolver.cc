#include "ts_gsolver.hh"

namespace tslam
{
    TSGSolver::TSGSolver() {};

    void TSGSolver::reconstruct(std::shared_ptr<tslam::TSTimber>& timber)
    {

        // TODO

        /* reconstruct algorithm
        ref1: polyFit (https://github.com/LiangliangNan/PolyFit)
        --- there is a licensed geometric solver Gurobi, this is not free and 
        we want to find another solution;

        Inspired by polyfit
        // 00. group the tags first(?)
        1. clip supporting planes with boundary box generated by all points (Open3d::bbox)
        0. create mesh planes scaled by 10e1000


        2. (refinement) merging similar planes (a: by proximity, b: by angle)

        3. intersection of planes

        4. selection of candidate rectangles (inbound points? might not be sufficient, we need to find an
        additional way to reinforce the selection of the candidate rectangles)

        5. join the rectangles to a polygon mesh
        5. check for  mesh sanity

        */
    }
}