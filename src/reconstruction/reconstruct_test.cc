#include <iostream>
#include "ts_plane_tag.hh"
#include "ts_timber.hh"
#include "ts_gsolver.hh"

#include <Eigen/Core>

#include <iostream>
#include <fstream>
#include <sstream>

#include <algorithm>

#include <stdexcept>

#include <open3d/Open3D.h>


//TODO: replace pre-compiled macro in cmake config
// #define PROFILER 1

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


bool ray2Plane(const Eigen::Vector3d &RayOrig,
               const Eigen::Vector3d &RayDir,
               const TSTPlane &Plane,
               float *OutT,
               float *OutVD)
{
    // Plane: ax+by+cz=d
    // Ray: P(t) = P0 + t * D
    // Intersection: P(t) = P0 + t * D = (x,y,z) = (a,b,c) * t = (a,b,c) * (d - (a*x+b*y+c*z)) / (a*a+b*b+c*c)
    // t = (d - (a*x+b*y+c*z)) / (a*a+b*b+c*c)
    const Eigen::Vector3d PlaneNormal(Plane.a, Plane.b, Plane.c);

    const double Denominator = PlaneNormal.dot(RayDir);
    if (Denominator == 0.0f)
    {
        // ray is parallel to plane
        return false;
    }

    const double Numerator = Plane.d - PlaneNormal.dot(RayOrig);
    const double t = Numerator / Denominator;

    if (OutT)
    {
        *OutT = t;
    }
    if (OutVD)
    {
        *OutVD = Denominator;
    }

    return true;
}

// Maximum out_point_count == 6, so out_points must point to 6-element array.
// out_point_count == 0 mean no intersection.
// out_points are not sorted.
void plane2AABBIntersect(const TSTPlane &plane,
                         const Eigen::Vector3d &aabb_min, 
                         const Eigen::Vector3d &aabb_max,
                         Eigen::Vector3d* out_points,
                         unsigned &out_point_count)
{
    out_point_count = 0;
    float vd, t;

    // Test edges along X axis, pointing right.
    Eigen::Vector3d dir = Eigen::Vector3d(aabb_max[0] - aabb_min[0], 0.f, 0.f);
    Eigen::Vector3d orig = aabb_min;
    if (ray2Plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
        out_points[out_point_count++] = orig + dir * t;
    orig = Eigen::Vector3d(aabb_min[0], aabb_max[1], aabb_min[2]);
    if (ray2Plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
        out_points[out_point_count++] = orig + dir * t;
    orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_max[2]);
    if (ray2Plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
        out_points[out_point_count++] = orig + dir * t;
    orig = Eigen::Vector3d(aabb_min[0], aabb_max[1], aabb_max[2]);
    if (ray2Plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
        out_points[out_point_count++] = orig + dir * t;

    // Test edges along Y axis, pointing up.
    dir = Eigen::Vector3d(0.f, aabb_max[1] - aabb_min[1], 0.f);
    orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_min[2]);
    if (ray2Plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
        out_points[out_point_count++] = orig + dir * t;
    orig = Eigen::Vector3d(aabb_max[0], aabb_min[1], aabb_min[2]);
    if (ray2Plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
        out_points[out_point_count++] = orig + dir * t;
    orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_max[2]);
    if (ray2Plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
        out_points[out_point_count++] = orig + dir * t;
    orig = Eigen::Vector3d(aabb_max[0], aabb_min[1], aabb_max[2]);
    if (ray2Plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
        out_points[out_point_count++] = orig + dir * t;

    // Test edges along Z axis, pointing forward.
    dir = Eigen::Vector3d(0.f, 0.f, aabb_max[2] - aabb_min[2]);
    orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_min[2]);
    if (ray2Plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
        out_points[out_point_count++] = orig + dir * t;
    orig = Eigen::Vector3d(aabb_max[0], aabb_min[1], aabb_min[2]);
    if (ray2Plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
        out_points[out_point_count++] = orig + dir * t;
    orig = Eigen::Vector3d(aabb_min[0], aabb_max[1], aabb_min[2]);
    if (ray2Plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
        out_points[out_point_count++] = orig + dir * t;
    orig = Eigen::Vector3d(aabb_max[0], aabb_max[1], aabb_min[2]);
    if (ray2Plane(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
        out_points[out_point_count++] = orig + dir * t;

    // Test the 8 vertices
    orig = aabb_min;
    if (plane.a * orig.x() + plane.b * orig.y() + plane.c * orig.z() + plane.d == 0.f)
        out_points[out_point_count++] = orig;
}

void sortIntersections(Eigen::Vector3d* points,
                       unsigned point_count,
                       const TSTPlane& plane)
{
    // sort points clockwise
    Eigen::Vector3d center = Eigen::Vector3d(0.f, 0.f, 0.f);
    for (unsigned i = 0; i < point_count; ++i)
        center += points[i];
    center /= (float)point_count;

    std::sort(points, points + point_count, [&center, &plane](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        Eigen::Vector3d v1 = a - center;
        Eigen::Vector3d v2 = b - center;
        float dot = v1.x() * v2.y() - v1.y() * v2.x();
        if (plane.a > 0.f)
            return dot > 0.f;
        else
            return dot < 0.f;
    });
}

std::vector<std::shared_ptr<open3d::geometry::Segment3D>> cropMeshPlaneByOBB(open3d::geometry::TriangleMesh& plane, open3d::geometry::AxisAlignedBoundingBox o3dOBB)
{

    std::vector<Eigen::Vector3d> planeCorners;
    for (auto& p : plane.vertices_)
    {
        planeCorners.push_back(p);
    }

    Eigen::Vector3d p1 = planeCorners[0];
    Eigen::Vector3d p2 = planeCorners[1];
    Eigen::Vector3d p3 = planeCorners[3];
    Eigen::Vector3d p4 = planeCorners[2];

    Eigen::Vector3d v1 = p2 - p1;
    Eigen::Vector3d v2 = p3 - p1;

    Eigen::Vector3d n = v1.cross(v2);
    n.normalize();

    double d = n.dot(p1);

    TSTPlane planeEq(n[0], n[1], n[2], d);




    //==================================================================================================
    // ref math: https://math.stackexchange.com/questions/4432127/intersection-between-segment-and-plane

    // std::vector<Eigen::Vector3d> planeIntersections;

    Eigen::Vector3d* outPtsPtr = new Eigen::Vector3d[3*6];
    unsigned int outPtsCount;

    plane2AABBIntersect(planeEq,
                        o3dOBB.min_bound_, 
                        o3dOBB.max_bound_,
                        outPtsPtr,
                        outPtsCount);


    // std::cout << "Number of intersections: " << std::to_string(outPtsCount) << std::endl;  // DEBUG


    // sort points by angle around the plane normal
    sortIntersections(outPtsPtr, outPtsCount, planeEq);





    // // get eigen vectors from the pointer
    std::vector<Eigen::Vector3d> planeIntersections;
    for (unsigned int i = 0; i < outPtsCount; i++)
    {
        planeIntersections.push_back(outPtsPtr[i]);
        // std::cout << "intersection: " << outPtsPtr[i].transpose() << std::endl;
    }
    // std::cout << "number of intersections: " << planeIntersections.size() << std::endl;







    //==================================================================================================


    std::vector<std::shared_ptr<open3d::geometry::Segment3D>> polygonSegments;
    // create a polygon from the intersection points
    for (int i = 0; i < planeIntersections.size(); i++)
    {
        polygonSegments.push_back(std::make_shared<open3d::geometry::Segment3D>(planeIntersections[i], planeIntersections[(i+1)%planeIntersections.size()]));
    }

    return polygonSegments;
}

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

int main()
{
    // create timber object & read yml TSlam map
    std::shared_ptr<tslam::TSTimber> timberPtr = std::make_shared<tslam::TSTimber>();
    const std::string FILENAME = "/home/as/TSlam/src/reconstruction/long_comb.yml";
    timberPtr->setPlaneTagsFromYAML(FILENAME);

    // create geometric solver
    tslam::TSGSolver solver = tslam::TSGSolver(timberPtr);



    //---------------------------------------------------------------------------------
    // Scale up planes
    //---------------------------------------------------------------------------------

    std::cout << "[DEBUG]: generting mesh planes from planeTags" << std::endl;                   // DEBUG
    solver.timeStart();

    const uint SCALE_PLN_FACTOR = 500;

    std::vector<open3d::geometry::TriangleMesh> meshPlnsScaledUp;
    for (auto& p : timberPtr->getPlaneTags())
    {
        open3d::geometry::TriangleMesh mPln = p.getOpen3dMesh();
        mPln.Scale(SCALE_PLN_FACTOR, p.getCenter());
        meshPlnsScaledUp.push_back(mPln);
    }

    solver.timeEnd();


    //---------------------------------------------------------------------------------
    // Cropping planes
    //---------------------------------------------------------------------------------

    std::cout << "[DEBUG]: cropping planes" << std::endl;                   // DEBUG
    auto start2 = std::chrono::high_resolution_clock::now();                                    // DEBUG
    // >>>> start code >>>>

    // get all the tag's center points in a point cloud, get its obb, scale it to half the scalePlnF and crop the planes
    open3d::geometry::PointCloud pntCld;
    for (auto& p : timberPtr->getPlaneTags())
    {
        pntCld.points_.push_back(p.getCenter());
    }

    // aabb
    const double SCALE_OBB_FACTOR = 2.0;  // const
    open3d::geometry::AxisAlignedBoundingBox aabb = pntCld.GetAxisAlignedBoundingBox();
    // open3d::geometry::OrientedBoundingBox obb = pntCld.GetOrientedBoundingBox();
    Eigen::Vector3d obbCenter = aabb.GetCenter();
    aabb.Scale(SCALE_OBB_FACTOR, obbCenter);


    std::vector<open3d::geometry::TriangleMesh> meshPlnsCrop;  //TODO: set as vector of points 
    std::vector<std::vector<std::shared_ptr<open3d::geometry::Segment3D>>> meshPlnsCropSegments;
    for (auto& plnF : meshPlnsScaledUp)
    {
        // open3d::geometry::TriangleMesh test = *plnF.Crop(obb);

        // open3d::geometry::TriangleMesh test = cropMeshPlaneByOBB(plnF, obb);
        // meshPlnsCrop.push_back(test);

        std::vector<std::shared_ptr<open3d::geometry::Segment3D>> test = cropMeshPlaneByOBB(plnF, aabb);


        meshPlnsCropSegments.push_back(test);
    }

    // return -1;  // DEBUG


    // <<<< end code <<<<
    auto end2 = std::chrono::system_clock::now();                                               // DEBUG
    std::chrono::duration<double> elapsed_seconds2 = end2-start2;                              // DEBUG
    std::cout << "[DEBUG]: elapsed time: " << elapsed_seconds2.count() << " s" << std::endl;  // DEBUG


    //##################################################################################
    // Debug visualizer
    //##################################################################################

    open3d::visualization::Visualizer* vis(new open3d::visualization::Visualizer());
    vis->CreateVisualizerWindow("TSPlaneTags", 1920, 1080);

    // draw base plane tags as wireframe
    for (auto& tag : timberPtr->getPlaneTags())
    {
        open3d::geometry::TriangleMesh tagBase = tag.getOpen3dMesh();
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
    // auto obbsegmentset = open3d::geometry::LineSet::CreateFromOrientedBoundingBox(obb);
    auto obbsegmentset = open3d::geometry::LineSet::CreateFromAxisAlignedBoundingBox(aabb);
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
        // break;  // DEBUG
    }

    vis->Run();
    vis->Close();
    vis->DestroyVisualizerWindow();

    return 0;
}