#include <iostream>
#include "ts_plane.hh"

#include <Eigen/Core>

#include <iostream>
#include <fstream>
#include <sstream>

#include <stdexcept>

// test files in https://drive.google.com/drive/folders/1wYFZq54syWwTFVQ5soJTMVUmcufcvQoT?usp=share_link
// if you have problem installing open3d:
// missing libc++ --> sudo apt install clang lldb lld


/*
[1] import and parse the yaml files into a vector of planes
[2] display those planes in a 3d window
*/


int main()
{
    // File location
    const std::string FILENAME = "/home/as/TSlam/src/reconstruction/long_comb.yml";

    std::vector<std::shared_ptr<tslam::TSPlane>> planes;
    tslam::TSPlane::parseFromMAPYAML(FILENAME, planes);

    // print all the planes
    for (uint i = 0; i < planes.size(); i++)
    {
        std::cout << *planes[i] << std::endl;
        std::cout << "corner A: " << planes[i]->getCornerA().transpose() << std::endl;
        std::cout << "corner B: " << planes[i]->getCornerB().transpose() << std::endl;
        std::cout << "corner C: " << planes[i]->getCornerC().transpose() << std::endl;
        std::cout << "corner D: " << planes[i]->getCornerD().transpose() << std::endl;
    }

    return 0;
}