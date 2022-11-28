#include <iostream>
#include "ts_plane_tag.hh"

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
[1] import and parse the yaml files into a vector of planeTags
[2] display those planeTags in a 3d window
*/


int main()
{
    const std::string FILENAME = "/home/as/TSlam/src/reconstruction/long_comb.yml";

    std::vector<std::shared_ptr<tslam::TSPlaneTag>> planeTags;
    tslam::TSPlaneTag::parseFromMAPYAML(FILENAME, planeTags);

    //---------------------------------------------------------------------------------
    // Debug visualizer
    //---------------------------------------------------------------------------------

    open3d::visualization::Visualizer* vis(new open3d::visualization::Visualizer());
    vis->CreateVisualizerWindow("TSPlaneTags", 1920, 1080);

    // draw plane tags as wireframe
    for (auto& tag : planeTags)
    {
        auto planeTagsLineset = open3d::geometry::LineSet::CreateFromTriangleMesh(*tag->toOpen3dMesh());
        planeTagsLineset->PaintUniformColor(Eigen::Vector3d(0, 1, 0.2));

        vis->AddGeometry(planeTagsLineset);
    }

    vis->Run();
    vis->Close();
    vis->DestroyVisualizerWindow();

    return 0;
}