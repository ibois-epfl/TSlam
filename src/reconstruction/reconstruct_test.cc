#include <iostream>
#include "ts_plane_tag.hh"
#include "ts_timber.hh"

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

//==================================================================================================

// test for https://asawicki.info/news_1428_finding_polygon_of_plane-aabb_intersection

struct TSTPlane
{
    TSTPlane(double a, double b, double c, double d)
        : a(a), b(b), c(c), d(d)
    {};
    double a, b, c, d;  // ax+by+cz=d
};


// OutVD > 0 means ray is back-facing the plane
// returns false if there is no intersection because ray is perpedicular to plane

bool ray_to_plane(const Eigen::Vector3d &RayOrig,
                  const Eigen::Vector3d &RayDir,
                  const TSTPlane &Plane,
                  float& OutT,
                  float& OutVD)
{
    OutVD = Plane.a * RayDir[0] + Plane.b * RayDir[1] + Plane.c * RayDir[2];
    if (OutVD == 0.0f)
        return false;
    OutT = - (Plane.a * RayOrig[0] + Plane.b * RayOrig[1] + Plane.c * RayOrig[2] + Plane.d) / OutVD;
    return true;
}

bool rayIntersectsPlane(const Eigen::Vector3d& rayOrigin,
                        const Eigen::Vector3d& rayDirection,
                        const TSTPlane& plane,
                        std::vector<Eigen::Vector3d>& intersectionPoints,
                        double& OutVD)
{
    double vd = plane.a * rayDirection(0) + plane.b * rayDirection(1) + plane.c * rayDirection(2);
    OutVD = vd;
    if (vd == 0)
    {
        return false;
    }
    double t = -(plane.a * rayOrigin(0) + plane.b * rayOrigin(1) + plane.c * rayOrigin(2) + plane.d) / vd;
    intersectionPoints.push_back(rayOrigin + t * rayDirection);
    return true;
}


// Maximum out_point_count == 6, so out_points must point to 6-element array.
// out_point_count == 0 mean no intersection.
// out_points are not sorted.
// void calc_plane_aabb_intersection_points(const TSTPlane &plane,
//                                          const Eigen::Vector3d &aabb_min, const
//                                          Eigen::Vector3d &aabb_max,
//                                          Eigen::Vector3d *out_points,
//                                          unsigned &out_point_count)
// {
//     out_point_count = 0;
//     float vd, t;

//     // Test edges along X axis, pointing right.
//     Eigen::Vector3d dir = Eigen::Vector3d(aabb_max[0] - aabb_min[0], 0.f, 0.f);
//     Eigen::Vector3d orig = aabb_min;
//     if (ray_to_plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
//         out_points[out_point_count++] = orig + dir * t;
//     orig = Eigen::Vector3d(aabb_min[0], aabb_max[1], aabb_min[2]);
//     if (ray_to_plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
//         out_points[out_point_count++] = orig + dir * t;
//     orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_max[2]);
//     if (ray_to_plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
//         out_points[out_point_count++] = orig + dir * t;
//     orig = Eigen::Vector3d(aabb_min[0], aabb_max[1], aabb_max[2]);
//     if (ray_to_plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
//         out_points[out_point_count++] = orig + dir * t;

//     // Test edges along Y axis, pointing up.
//     dir = Eigen::Vector3d(0.f, aabb_max[1] - aabb_min[1], 0.f);
//     orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_min[2]);
//     if (ray_to_plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
//         out_points[out_point_count++] = orig + dir * t;
//     orig = Eigen::Vector3d(aabb_max[0], aabb_min[1], aabb_min[2]);
//     if (ray_to_plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
//         out_points[out_point_count++] = orig + dir * t;
//     orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_max[2]);
//     if (ray_to_plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
//         out_points[out_point_count++] = orig + dir * t;
//     orig = Eigen::Vector3d(aabb_max[0], aabb_min[1], aabb_max[2]);
//     if (ray_to_plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
//         out_points[out_point_count++] = orig + dir * t;

//     // Test edges along Z axis, pointing forward.
//     dir = Eigen::Vector3d(0.f, 0.f, aabb_max[2] - aabb_min[2]);
//     orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_min[2]);
//     if (ray_to_plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
//         out_points[out_point_count++] = orig + dir * t;
//     orig = Eigen::Vector3d(aabb_max[0], aabb_min[1], aabb_min[2]);
//     if (ray_to_plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
//         out_points[out_point_count++] = orig + dir * t;
//     orig = Eigen::Vector3d(aabb_min[0], aabb_max[1], aabb_min[2]);
//     if (ray_to_plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
//         out_points[out_point_count++] = orig + dir * t;
//     orig = Eigen::Vector3d(aabb_max[0], aabb_max[1], aabb_min[2]);
//     if (ray_to_plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
//         out_points[out_point_count++] = orig + dir * t;
// }

std::vector<std::shared_ptr<open3d::geometry::Segment3D>> cropMeshPlaneByOBB(open3d::geometry::TriangleMesh& plane, open3d::geometry::OrientedBoundingBox o3dOBB)
{

    std::vector<Eigen::Vector3d> OBBcorners = o3dOBB.GetBoxPoints();

    // get lines of the OBB from Open3d structure
    ///      ------- x
    ///     /|
    ///    / |
    ///   /  | z
    ///  y
    ///      0 ------------------- 1
    ///       /|                /|
    ///      / |               / |
    ///     /  |              /  |
    ///    /   |             /   |
    /// 2 ------------------- 7  |
    ///   |    |____________|____| 6
    ///   |   /3            |   /
    ///   |  /              |  /
    ///   | /               | /
    ///   |/                |/
    /// 5 ------------------- 4
    // std::vector<Eigen::Vector3d> OBBsegments;
    std::vector<std::shared_ptr<open3d::geometry::Segment3D>> OBBsegments;
    OBBsegments.push_back(std::make_shared<open3d::geometry::Segment3D>(OBBcorners[0], OBBcorners[1]));
    OBBsegments.push_back(std::make_shared<open3d::geometry::Segment3D>(OBBcorners[0], OBBcorners[1]));
    OBBsegments.push_back(std::make_shared<open3d::geometry::Segment3D>(OBBcorners[1], OBBcorners[7]));
    OBBsegments.push_back(std::make_shared<open3d::geometry::Segment3D>(OBBcorners[7], OBBcorners[2]));
    OBBsegments.push_back(std::make_shared<open3d::geometry::Segment3D>(OBBcorners[2], OBBcorners[0]));

    OBBsegments.push_back(std::make_shared<open3d::geometry::Segment3D>(OBBcorners[0], OBBcorners[3]));
    OBBsegments.push_back(std::make_shared<open3d::geometry::Segment3D>(OBBcorners[1], OBBcorners[6]));
    OBBsegments.push_back(std::make_shared<open3d::geometry::Segment3D>(OBBcorners[2], OBBcorners[5]));
    OBBsegments.push_back(std::make_shared<open3d::geometry::Segment3D>(OBBcorners[7], OBBcorners[4]));

    OBBsegments.push_back(std::make_shared<open3d::geometry::Segment3D>(OBBcorners[3], OBBcorners[6]));
    OBBsegments.push_back(std::make_shared<open3d::geometry::Segment3D>(OBBcorners[6], OBBcorners[4]));
    OBBsegments.push_back(std::make_shared<open3d::geometry::Segment3D>(OBBcorners[4], OBBcorners[5]));
    OBBsegments.push_back(std::make_shared<open3d::geometry::Segment3D>(OBBcorners[5], OBBcorners[3]));



    // TODO: this has probably to be replaced by a tagplane object (not using open3d mesh as input)
    // intersect each line with the plane


    // std::cout << "plane vertices" << std::to_string(plane.vertices_.size()) << std::endl;  // DEBUG


    std::vector<Eigen::Vector3d> planeCorners;
    for (auto& p : plane.vertices_)
    {
        planeCorners.push_back(p);
    }

    // plane.ComputeVertexNormals();

    // convert plane to ax+by+cz=d
    std::vector<std::shared_ptr<open3d::geometry::Segment3D>> planeDebug;  // DEBUG


    Eigen::Vector3d p1 = planeCorners[0];
    Eigen::Vector3d p2 = planeCorners[1];
    Eigen::Vector3d p3 = planeCorners[3];
    Eigen::Vector3d p4 = planeCorners[2];

    planeDebug.push_back(std::make_shared<open3d::geometry::Segment3D>(p1, p2)); // DEBUG
    planeDebug.push_back(std::make_shared<open3d::geometry::Segment3D>(p1, p3)); // DEBUG
    planeDebug.push_back(std::make_shared<open3d::geometry::Segment3D>(p2, p4)); // DEBUG
    planeDebug.push_back(std::make_shared<open3d::geometry::Segment3D>(p3, p4)); // DEBUG

    Eigen::Vector3d v1 = p2 - p1;
    Eigen::Vector3d v2 = p3 - p1;


    Eigen::Vector3d n = v1.cross(v2);


    // n.normalize();

    planeDebug.push_back(std::make_shared<open3d::geometry::Segment3D>(p1, n/1000 + p1)); // DEBUG


    double d = n.dot(p1);

    TSTPlane planeEq(n[0], n[1], n[2], d);


    std::cout << "POP1" << std::endl;  // DEBUG


    //==================================================================================================
    // ref math: https://math.stackexchange.com/questions/4432127/intersection-between-segment-and-plane
    // ray casting open3d: http://www.open3d.org/html/cpp_api/classopen3d_1_1t_1_1geometry_1_1_raycasting_scene.html

    std::vector<Eigen::Vector3d> planeIntersections;


    // create raycasting scene from open3d







    // // Maximum out_point_count == 6, so out_points must point to 6-element array
    // // out_point_count == 0 mean no intersection
    // // out_points are not sorted

    double outVD;
    // Eigen::Vector3d intersection;
    for (auto& seg : OBBsegments)
    {
        // detect if there is collision between segment and plance
        if (rayIntersectsPlane(seg->Origin(), Eigen::Vector3d(seg->EndPoint()-seg->Origin()), planeEq, planeIntersections, outVD))
        {
            std::cout << "collision detected" << std::endl;
        }
    }

    std::cout << "number of intersections: " << planeIntersections.size() << std::endl;

    // std::cout << "POP2" << std::endl;  // DEBUG






    //==================================================================================================


    std::vector<std::shared_ptr<open3d::geometry::Segment3D>> polygonSegments;
    // create a polygon from the intersection points
    for (int i = 0; i < planeIntersections.size(); i++)
    {
        int j = (i + 1) % planeIntersections.size();
        polygonSegments.push_back(std::make_shared<open3d::geometry::Segment3D>(planeIntersections[i], planeIntersections[j]));
    }

    // return polygonSegments;

    return planeDebug;
}


int main()
{
    const std::string FILENAME = "/home/as/TSlam/src/reconstruction/long_comb.yml";

    tslam::TSTimber timber = tslam::TSTimber();
    timber.setPlaneTagsFromYAML(FILENAME);
    // std::vector<std::shared_ptr<tslam::TSPlaneTag>> planeTags;
    // tslam::TSPlaneTag::parseFromMAPYAML(FILENAME, planeTags);


    //---------------------------------------------------------------------------------
    // Reconstruction
    //---------------------------------------------------------------------------------

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

    //---------------------------------------------------------------------------------
    // Scale up planes
    //---------------------------------------------------------------------------------

    std::cout << "[DEBUG]: generting mesh planes from planeTags" << std::endl;                   // DEBUG
    auto start1 = std::chrono::high_resolution_clock::now();                                    // DEBUG
    // >>>> start code >>>>

    const uint SCALE_PLN_FACTOR = 500;

    std::vector<open3d::geometry::TriangleMesh> meshPlnsScaledUp;
    for (auto& p : timber.getPlaneTags())
    {
        open3d::geometry::TriangleMesh mPln = p->getOpen3dMesh();
        mPln.Scale(SCALE_PLN_FACTOR, p->getCenter());
        meshPlnsScaledUp.push_back(mPln);
    }

    // <<<< end code <<<<
    auto end1 = std::chrono::system_clock::now();                                               // DEBUG
    std::chrono::duration<double> elapsed_seconds1 = end1-start1;                              // DEBUG
    std::cout << "[DEBUG]: elapsed time: " << elapsed_seconds1.count() << " s" << std::endl;  // DEBUG


    //---------------------------------------------------------------------------------
    // Cropping planes
    //---------------------------------------------------------------------------------

    std::cout << "[DEBUG]: generting mesh planes from planeTags" << std::endl;                   // DEBUG
    auto start2 = std::chrono::high_resolution_clock::now();                                    // DEBUG
    // >>>> start code >>>>

    // get all the tag's center points in a point cloud, get its obb, scale it to half the scalePlnF and crop the planes
    open3d::geometry::PointCloud pntCld;
    for (auto& p : timber.getPlaneTags())
    {
        pntCld.points_.push_back(p->getCenter());
    }

    const double SCALE_OBB_FACTOR = 2.0;  // const
    open3d::geometry::OrientedBoundingBox obb = pntCld.GetOrientedBoundingBox();
    Eigen::Vector3d obbCenter = obb.GetCenter();
    obb.Scale(SCALE_OBB_FACTOR, obbCenter);

    std::vector<open3d::geometry::TriangleMesh> meshPlnsCrop;  //TODO: set as vector of points 
    std::vector<std::vector<std::shared_ptr<open3d::geometry::Segment3D>>> meshPlnsCropSegments;
    for (auto& plnF : meshPlnsScaledUp)
    {
        // open3d::geometry::TriangleMesh test = *plnF.Crop(obb);

        // open3d::geometry::TriangleMesh test = cropMeshPlaneByOBB(plnF, obb);
        // meshPlnsCrop.push_back(test);

        std::vector<std::shared_ptr<open3d::geometry::Segment3D>> test = cropMeshPlaneByOBB(plnF, obb);


        meshPlnsCropSegments.push_back(test);
    }

    // return -1;  // DEBUG


    // <<<< end code <<<<
    auto end2 = std::chrono::system_clock::now();                                               // DEBUG
    std::chrono::duration<double> elapsed_seconds2 = end2-start2;                              // DEBUG
    std::cout << "[DEBUG]: elapsed time: " << elapsed_seconds2.count() << " s" << std::endl;  // DEBUG


    //##################################################################################
    // Debug visualizer
    //---------------------------------------------------------------------------------

    open3d::visualization::Visualizer* vis(new open3d::visualization::Visualizer());
    vis->CreateVisualizerWindow("TSPlaneTags", 1920, 1080);

    // draw base plane tags as wireframe
    for (auto& tag : timber.getPlaneTags())
    {
        open3d::geometry::TriangleMesh tagBase = tag->getOpen3dMesh();
        auto planeTagsLineset1 = open3d::geometry::LineSet::CreateFromTriangleMesh(tagBase);
        planeTagsLineset1->PaintUniformColor(Eigen::Vector3d(1, 1, 0.2));
        vis->AddGeometry(planeTagsLineset1);
    }

    // // draw base scaled up tags as wireframe
    // for (auto& tag : timber.getPlaneTags())
    // {
    //     open3d::geometry::TriangleMesh tagScaledUp = tag->getOpen3dMesh();

    //     tagScaledUp.Scale(500, tag->getCenter());

    //     auto planeTagsLineset2 = open3d::geometry::LineSet::CreateFromTriangleMesh(tagScaledUp);
    //     planeTagsLineset2->PaintUniformColor(Eigen::Vector3d(0, 1, 0.2));
    //     vis->AddGeometry(planeTagsLineset2);
    // }

    // add bounding box
    auto obbsegmentset = open3d::geometry::LineSet::CreateFromOrientedBoundingBox(obb);
    obbsegmentset->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
    vis->AddGeometry(obbsegmentset);

    // draw scaled and cropped planes
    for (auto& pln : meshPlnsCrop)
    {
        auto planeTagsLineset3 = open3d::geometry::LineSet::CreateFromTriangleMesh(pln);
        planeTagsLineset3->PaintUniformColor(Eigen::Vector3d(0, 0, 1));
        vis->AddGeometry(planeTagsLineset3);
    }

    // draw polygon segments3D 
    for (auto& vecSegm : meshPlnsCropSegments)
    {
        for (auto& segm : vecSegm)
        {
            // draw spheres at the segment's endpoints
            std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
            // Create a LineSet from given points and line indices
            segLineset->points_.push_back(segm->Origin());
            segLineset->points_.push_back(segm->EndPoint());
            segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
            segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
            segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
            segLineset->PaintUniformColor(Eigen::Vector3d(0.5, 1, 0.5));
            vis->AddGeometry(segLineset);

            // auto segmLineset = open3d::geometry::LineSet::CreateFromSegment3D(*segm);
            // segmLineset->PaintUniformColor(Eigen::Vector3d(0, 1, 0));
            // vis->AddGeometry(segm);
        }
        break;  // DEBUG
    }

    vis->Run();
    vis->Close();
    vis->DestroyVisualizerWindow();

    return 0;
}