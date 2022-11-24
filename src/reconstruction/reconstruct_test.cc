#include <iostream>
#include "ts_plane.hh"

#include <Eigen/Core>

#include <iostream>
#include <fstream>
#include <sstream>

#include <stdexcept>

#include <open3d/Open3D.h>

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

    // draw planes as filar with open3d
    std::vector<tslam::o3dMeshPtr> meshes;
    for (auto& plane : planes)
    {
        meshes.push_back(plane->toOpen3dMesh());
    }

    open3d::visualization::Visualizer* vis(new open3d::visualization::Visualizer());
    vis->CreateVisualizerWindow("TSPlanes", 1920, 1080);
    for (auto& mesh : meshes)
    {
        vis->AddGeometry(mesh);
    }
    vis->Run();
    vis->DestroyVisualizerWindow();




    return 0;
}