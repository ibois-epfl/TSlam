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


    //---------------------------------------------------------------------------------
    // Debug visualizer
    //---------------------------------------------------------------------------------

    open3d::visualization::Visualizer* vis(new open3d::visualization::Visualizer());
    vis->CreateVisualizerWindow("TSPlanes", 1920, 1080);

    // draw plane tags as wireframe
    for (auto& plane : planes)
    {
        auto planesLineset = open3d::geometry::LineSet::CreateFromTriangleMesh(*plane->toOpen3dMesh());
        planesLineset->PaintUniformColor(Eigen::Vector3d(0, 1, 0.2));

        vis->AddGeometry(planesLineset);
    }

    vis->Run();
    vis->Close();
    vis->DestroyVisualizerWindow();

    return 0;
}