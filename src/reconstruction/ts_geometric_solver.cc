#include "ts_geometric_solver.hh"
#include <open3d/Open3D.h>
#include <stdexcept>
#include <algorithm>
#include <math.h>


namespace tslam
{
    void TSGeometricSolver::reconstruct()
    {
        this->rDetectCreasesTags();
        this->rIntersectTagPlnAABB();



        // // =======================================================================================
        // // =======================================================================================

        // build a kdtree with radius search of the polygons centers
        open3d::geometry::KDTreeFlann kdtree;
        std::shared_ptr<open3d::geometry::PointCloud> pntCldPCtr = std::make_shared<open3d::geometry::PointCloud>();
        std::vector<tslam::TSTPlane> planes;
        for (auto& p : this->m_PlnAABBPolygons)
        {
            pntCldPCtr->points_.push_back(p.getCenter());
        }


        kdtree.SetGeometry(*pntCldPCtr);


        // for each polygon center, find the closest polygon centers by radius search

        
        double radius = 3.;  ///<--- ! this has to be parametrized in the class
        std::vector<int> indices;
        std::vector<double> distances;

        std::shared_ptr<open3d::geometry::PointCloud> pntCldNewPlanes = std::make_shared<open3d::geometry::PointCloud>();
        std::vector<tslam::TSTPlane> newPlanes;
        std::vector<int> mergedIndices;

        // keep track of the indices of the polygons that have been merged

        for (int i=0; i < pntCldPCtr->points_.size(); i++)
        {
            kdtree.SearchRadius(pntCldPCtr->points_[i], radius, indices, distances);
            // std::cout << "Number of neighbors: " << indices.size() << std::endl;

            // keep only one of all the neighbors by merging them
            int k = 0;
            Eigen::Vector3d meanPt(0, 0, 0);
            Eigen::Vector3d meanNorm(0, 0, 0);
            for (int j=0; j < indices.size(); j++)
            {
                // check if the index has already been merged
                if (std::find(mergedIndices.begin(), mergedIndices.end(), indices[j]) != mergedIndices.end())
                {
                    // if it has, check the next one
                    continue;
                }
                else
                {
                    // if it hasn't,
                    // mean the center of the polygons
                    meanPt += pntCldPCtr->points_[indices[j]];

                    // get the TSPlanes of the polygons and mean their normals
                    meanNorm += this->m_PlnAABBPolygons[indices[j]].getNormal();

                    // add the index to the merged indices
                    mergedIndices.push_back(indices[j]);
                    k++;
                }
                // mean the indices of all the neighbors and create a new point
            }
            meanPt /= k;
            meanNorm /= k;



            if (k > 1)
            {
                pntCldNewPlanes->points_.push_back(meanPt);  // DEBUG

                TSTPlane newPlane(meanNorm, meanPt);
                newPlanes.push_back(newPlane);
            }
        }


        // // =======================================================================================
        // // =======================================================================================

        // TEST: rerunning the AABB plane interesections but now with the new mean planes
        std::vector<TSPolygon> outputSimpPlnAABBPolygons;

        Eigen::Vector3d* outPtsPtr2 = new Eigen::Vector3d[3*6];
        std::vector<Eigen::Vector3d>* planeIntersections2 = new std::vector<Eigen::Vector3d>();
        for (auto& pln : newPlanes)
        {
            
            // b. caculate the intersection points
            unsigned int outPtsCount2;
            this->rPlane2AABBSegmentIntersect(pln,
                                              this->m_Timber->getAABB().min_bound_, 
                                              this->m_Timber->getAABB().max_bound_,
                                              outPtsPtr2,
                                              outPtsCount2);
            this->rSortIntersectionPoints(outPtsPtr2, outPtsCount2, pln);

            // c. save the result into a polygon
            planeIntersections2->reserve(outPtsCount2);
            planeIntersections2->clear();
            for (unsigned int i = 0; i < outPtsCount2; i++)
            {
                planeIntersections2->push_back(outPtsPtr2[i]);
            }
            outputSimpPlnAABBPolygons.push_back(TSPolygon(*planeIntersections2, pln));
        }
        delete outPtsPtr2;
        delete planeIntersections2;






#ifdef TSLAM_REC_DEBUG
    // Debug visualizer
    open3d::visualization::Visualizer* vis(new open3d::visualization::Visualizer());
    vis->CreateVisualizerWindow("TSPlaneTags", 1920, 1080);

    // draw base plane tags as wireframe
    for (auto& tag : this->m_Timber->getPlaneTags())
    {
        open3d::geometry::TriangleMesh tagBase = tag.getOpen3dMesh();
        auto planeTagsLineset1 = open3d::geometry::LineSet::CreateFromTriangleMesh(tagBase);
        if (tag.isEdge())
            tag.setColor(Eigen::Vector3d(0, 1, 0));
        else
            tag.setColor(Eigen::Vector3d(1, 0, 0));
        planeTagsLineset1->PaintUniformColor(tag.getColor());
        vis->AddGeometry(planeTagsLineset1);
    }

    // draw tag centers as point cloud with normals
    this->m_Timber->getTagsCtrs().PaintUniformColor(Eigen::Vector3d(0, 0, 0));
    vis->AddGeometry(std::make_shared<open3d::geometry::PointCloud>(this->m_Timber->getTagsCtrs()));

    // // show point cloud normals
    // for (int i=0; i < this->m_Timber->getTagsCtrs().points_.size(); i++)
    // {
    //     auto p = this->m_Timber->getTagsCtrs().points_[i];
    //     auto n = this->m_Timber->getTagsCtrs().normals_[i];
    //     auto line = std::make_shared<open3d::geometry::LineSet>();
    //     line->points_.push_back(p);
    //     line->points_.push_back(p + n);
    //     line->lines_.push_back(Eigen::Vector2i(0, 1));
    //     line->colors_.push_back(Eigen::Vector3d(1, 0, 0));
    //     line->colors_.push_back(Eigen::Vector3d(1, 0, 0));
    //     line->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
    //     vis->AddGeometry(line);
    // }


    // // add bounding box
    // auto obbsegmentset = open3d::geometry::LineSet::CreateFromAxisAlignedBoundingBox(this->m_Timber->getAABB());
    // obbsegmentset->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
    // vis->AddGeometry(obbsegmentset);

    // draw polygon segments3D 
    for (auto& pg : this->m_PlnAABBPolygons)
    {
        std::vector<Eigen::Vector3d> pts = pg.getPoints();
        for (int i = 0; i < pts.size(); i++)
        {
            std::shared_ptr<open3d::geometry::Segment3D> segm = std::make_shared<open3d::geometry::Segment3D>(pts[i], pts[(i+1)%pts.size()]);
            std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
            segLineset->points_.push_back(segm->Origin());
            segLineset->points_.push_back(segm->EndPoint());
            segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
            segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
            segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
            segLineset->PaintUniformColor(Eigen::Vector3d(0.5, 1, 0.5));
            vis->AddGeometry(segLineset);
        }
    }

    // draw polygon centers
    pntCldPCtr->PaintUniformColor(Eigen::Vector3d(0, 1, 1));
    vis->AddGeometry(pntCldPCtr);

    // draw new merged planes centers
    pntCldNewPlanes->PaintUniformColor(Eigen::Vector3d(0.7, 0.3, 0.9));
    vis->AddGeometry(pntCldNewPlanes);


    // draw new interesected polygons
    for (auto& pg : outputSimpPlnAABBPolygons)
    {
        std::vector<Eigen::Vector3d> pts = pg.getPoints();
        for (int i = 0; i < pts.size(); i++)
        {
            std::shared_ptr<open3d::geometry::Segment3D> segm = std::make_shared<open3d::geometry::Segment3D>(pts[i], pts[(i+1)%pts.size()]);
            std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
            segLineset->points_.push_back(segm->Origin());
            segLineset->points_.push_back(segm->EndPoint());
            segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
            segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
            segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
            segLineset->PaintUniformColor(Eigen::Vector3d(0., 0., 1.));
            vis->AddGeometry(segLineset);
        }
    }

    vis->Run();
    vis->Close();
    vis->DestroyVisualizerWindow();
#endif
    }

    void TSGeometricSolver::rDetectCreasesTags()
    {
        open3d::geometry::KDTreeFlann kdtree;
        kdtree.SetGeometry(open3d::geometry::PointCloud(this->m_Timber->getTagsCtrs()));

        std::vector<int> indices;
        std::vector<double> distances;
        const int knn = 2;

        for (int i = 0; i < this->m_Timber->getPlaneTags().size(); i++)
        {
            kdtree.SearchKNN(this->m_Timber->getPlaneTags()[i].getCenter(), knn, indices, distances);

            double angle = this->rAngleBetweenVectors(this->m_Timber->getPlaneTags()[i].getNormal(), 
                                                      this->m_Timber->getPlaneTags()[indices[1]].getNormal());

            if (angle < this->m_CreaseAngleThreshold) 
            {
                this->m_Timber->getPlaneTags()[i].setType(TSRTagType::Face);
            }
            else
            {
                this->m_Timber->getPlaneTags()[i].setType(TSRTagType::Edge);
            }
        }
    }
    double TSGeometricSolver::rAngleBetweenVectors(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
    {
        double angle = std::acos(v1.dot(v2) / (v1.norm() * v2.norm()));
        return (angle * 180 / M_PI);
    }

    void TSGeometricSolver::rIntersectTagPlnAABB()
    {
        this->m_Timber->scaleAABB(this->m_AABBScaleFactor);

        this->timeStart("[PROFILER]: intersect planes with AABB");

        Eigen::Vector3d* outPtsPtr = new Eigen::Vector3d[3*6];
        std::vector<Eigen::Vector3d>* planeIntersections = new std::vector<Eigen::Vector3d>();
        for (auto& t : this->m_Timber->getPlaneTags())
        {
            // a. skip face tags
            if (t.isFace()) continue;
            
            // b. caculate the intersection points
            unsigned int outPtsCount;
            this->rPlane2AABBSegmentIntersect(t.getPlane(),
                                              this->m_Timber->getAABB().min_bound_, 
                                              this->m_Timber->getAABB().max_bound_,
                                              outPtsPtr,
                                              outPtsCount);
            this->rSortIntersectionPoints(outPtsPtr, outPtsCount, t.getPlane());

            // c. save the result into a polygon
            planeIntersections->reserve(outPtsCount);
            planeIntersections->clear();
            for (unsigned int i = 0; i < outPtsCount; i++)
            {
                planeIntersections->push_back(outPtsPtr[i]);
            }
            this->m_PlnAABBPolygons.push_back(TSPolygon(*planeIntersections, t.getPlane()));
        }
        delete outPtsPtr;
        delete planeIntersections;


        this->timeEnd();
    }
    bool TSGeometricSolver::rRay2PlaneIntersection(const Eigen::Vector3d &RayOrig,
                                                   const Eigen::Vector3d &RayDir,
                                                   const TSTPlane &Plane,
                                                   float *OutT,
                                                   float *OutVD)
    {
        const Eigen::Vector3d PlaneNormal(Plane.a, Plane.b, Plane.c);  /// FIXME: change to Getters from TSPlane

        const double Denominator = PlaneNormal.dot(RayDir);
        if (Denominator == 0.0f)  return false;  // ray is parallel to plane

        const double Numerator = Plane.d - PlaneNormal.dot(RayOrig);
        const double t = Numerator / Denominator;

        if (OutT) *OutT = t;
        if (OutVD) *OutVD = Denominator;

        return true;
    }
    void TSGeometricSolver::rPlane2AABBSegmentIntersect(const TSTPlane &plane,
                                                        const Eigen::Vector3d &aabb_min, 
                                                        const Eigen::Vector3d &aabb_max,
                                                        Eigen::Vector3d* out_points,
                                                        unsigned &out_point_count)
    {
        out_point_count = 0;
        float vd, t;

        // Test edges along X axis, pointing right
        Eigen::Vector3d dir = Eigen::Vector3d(aabb_max[0] - aabb_min[0], 0.f, 0.f);
        Eigen::Vector3d orig = aabb_min;
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_min[0], aabb_max[1], aabb_min[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_max[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_min[0], aabb_max[1], aabb_max[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;

        // Test edges along Y axis, pointing up
        dir = Eigen::Vector3d(0.f, aabb_max[1] - aabb_min[1], 0.f);
        orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_min[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_max[0], aabb_min[1], aabb_min[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_max[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_max[0], aabb_min[1], aabb_max[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;

        // Test edges along Z axis, pointing forward
        dir = Eigen::Vector3d(0.f, 0.f, aabb_max[2] - aabb_min[2]);
        orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_min[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_max[0], aabb_min[1], aabb_min[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_min[0], aabb_max[1], aabb_min[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_max[0], aabb_max[1], aabb_min[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;

        // Test the 8 vertices
        orig = aabb_min;
        if (plane.a * orig.x() + plane.b * orig.y() + plane.c * orig.z() + plane.d == 0.f)
            out_points[out_point_count++] = orig;
    }
    void TSGeometricSolver::rSortIntersectionPoints(Eigen::Vector3d* points,
                                                    unsigned point_count,
                                                    const TSTPlane& plane)
    {
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

    bool TSGeometricSolver::check4PlaneTags()
    {
        std::vector<tslam::TSRTag> tstag = this->m_Timber->getPlaneTags();
        if (tstag.size() != 0) return true;
        else
        {
            throw std::runtime_error("[ERROR]: corners are not set.");
        }
        return false;
    }
}