#include "ts_geometric_solver.hh"
#include <open3d/Open3D.h>
#include <stdexcept>
#include <algorithm>
#include <math.h>
#include <map>
#include <numeric>
#include <algorithm>
#include <iterator>


namespace tslam
{
    void TSGeometricSolver::reconstruct()
    {
        this->rDetectCreasesTags();
        this->rIntersectTagPlnAABB();
        this->rIntersectMeanPolygonPlnAABB();

        // // =============================================================================
        // testing
        // // =============================================================================

        // // intersection between two segments
        // TSSegment segT1 = TSSegment(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
        // TSSegment segT2 = TSSegment(Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(1, 0, 1));

        // Eigen::Vector3d* intersectPtT = new Eigen::Vector3d();
        // bool isIntersectTRes = segT1.intersect(segT2, *intersectPtT);
        // std::cout << "isIntersect: " << isIntersectTRes << std::endl;

        // ask for user input
        // std::cout << "Press ENTER to continue..." << std::endl;
        // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), ' ');

        // return;


        // // =============================================================================
        // intersect polygons and get segments connecting the intersection points
        // // =============================================================================

        std::vector<TSSegment> splitSegs;
        this->rIntersectPolygons(this->m_MergedPolygons, splitSegs);
        std::cout << "[DEBUG] Number of split segments: " << splitSegs.size() << std::endl;  // DEBUG


        // // // =============================================================================
        // // split poly with every seg until no more splits are possible
        // // // =============================================================================



        std::vector<TSPolygon> splitPolygons = this->m_MergedPolygons;
        uint NMergedPolygons = this->m_MergedPolygons.size();
        uint NIntersectSegments = splitSegs.size();
        uint maxNSplitPolygons = NIntersectSegments * 2 * NMergedPolygons;
        std::cout << "[DEBUG] max number of split polygons: " << maxNSplitPolygons << std::endl;

        std::vector<bool> flagsErase;


        std::vector<TSPolygon> splitPolygonsReserve;


        bool isSplittable = true;
        bool isSplit = false;


        std::tuple<TSPolygon, TSPolygon> splitPolys;


        while (isSplittable)
        {

            for (auto& sPoly : splitPolygons)
            {
                std::cout << "==============================================" << std::endl;
                std::cout << "split polygon size: " << splitPolygons.size() << std::endl;

                uint Nsplits = 0;

                for (auto& seg : splitSegs)
                {
                    
                    isSplit = sPoly.splitPolygon(seg, splitPolys);
                    //  std::cout << "[DEBUG] isSplit: " << isSplit << std::endl;

                    if (isSplit)
                    {
                        splitPolygonsReserve.emplace_back(std::get<0>(splitPolys));
                        splitPolygonsReserve.emplace_back(std::get<1>(splitPolys));

                        Nsplits++;
                    }
                }

                // mark the polygon as split, hence to be erased from the next cycle
                if (Nsplits > 0)
                {
                    flagsErase.push_back(true);
                }
                else
                {
                    flagsErase.push_back(false);
                }
            }


            // EXIT CONDITION: stops when there is no more split == when the vector of bool split flags is all false
            if ( std::adjacent_find( flagsErase.begin(), flagsErase.end(), std::not_equal_to<>() ) == flagsErase.end() )
            {
                std::cout << "All elements are equal each other" << std::endl;
                std::cout << "[DEBUG] hit exit condition!" << std::endl;
                std::cout << "[DEBUG] exit number of split poly: " << splitPolygons.size() << std::endl;

                isSplittable = false;
                // break;
            }

            // erase the polygons that have been split
            for (int j = 0; j < flagsErase.size(); j++)
            {
                if (flagsErase[j])
                {
                    splitPolygons.erase(splitPolygons.begin() + j);
                }
            }
            flagsErase.clear();


            // add the new split polgons to the vector
            for (auto& sPoly : splitPolygonsReserve)
            {
                splitPolygons.push_back(sPoly);
            }

            // std::cout << "[DEBUG] SplitPolygon size --inloop--: " << splitPolygons.size() << std::endl;


            // check if there no more polygons to split

        }

        std::cout << "[DEBUG] MergedPoly number: " << this->m_MergedPolygons.size() << std::endl;
        std::cout << "[DEBUG] Splitting segments number: " << splitSegs.size() << std::endl;
        std::cout << "[DEBUG] Split Polygons number: " << splitPolygons.size() << std::endl;


        // =============================================================================



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

    // draw AABB
    std::shared_ptr<open3d::geometry::LineSet> aabbLineset = std::make_shared<open3d::geometry::LineSet>();

    // draw polygon segments3D 
    // for (auto& pg : this->m_PlnAABBPolygons)
    // {
    //     std::vector<Eigen::Vector3d> pts = pg.getPoints();
    //     for (int i = 0; i < pts.size(); i++)
    //     {
    //         std::shared_ptr<open3d::geometry::Segment3D> segm = std::make_shared<open3d::geometry::Segment3D>(pts[i], pts[(i+1)%pts.size()]);
    //         std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
    //         segLineset->points_.push_back(segm->Origin());
    //         segLineset->points_.push_back(segm->EndPoint());
    //         segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
    //         segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //         segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //         segLineset->PaintUniformColor(Eigen::Vector3d(0.5, 1, 0.5));
    //         vis->AddGeometry(segLineset);
    //     }
    // }

    // draw first intersection polygon centers
    for (auto& fpoly : this->m_PlnAABBPolygons)
    {
        std::shared_ptr<open3d::geometry::PointCloud> pntCldPCtr = std::make_shared<open3d::geometry::PointCloud>();
        pntCldPCtr->points_.push_back(fpoly.getCenter());
        pntCldPCtr->PaintUniformColor(Eigen::Vector3d(0, 1, 1));
        vis->AddGeometry(pntCldPCtr);
    }

    // draw new merged planes centers
    std::shared_ptr<open3d::geometry::PointCloud> pntCldNewPlanes = std::make_shared<open3d::geometry::PointCloud>();
    for (auto& pg : this->m_MergedPolygons)
    {
        pntCldNewPlanes->points_.push_back(pg.getCenter());
        pntCldNewPlanes->PaintUniformColor(Eigen::Vector3d(0.7, 0.3, 0.9));
        vis->AddGeometry(pntCldNewPlanes);
    }

    // draw new interesected polygons
    for (auto& pg : this->m_MergedPolygons)
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


    // draw splitting segments
    for (auto& seg : splitSegs)
    {
        std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
        segLineset->points_.push_back(seg.Origin());
        segLineset->points_.push_back(seg.EndPoint());
        segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
        segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
        segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
        segLineset->PaintUniformColor(Eigen::Vector3d(1., 0., 1.));
        vis->AddGeometry(segLineset);
    }
        
    // // // show split polygons
    // for (auto& pg : splitPolygons)
    // {
    //     std::vector<Eigen::Vector3d> pts = pg.getPoints();
    //     for (int i = 0; i < pts.size(); i++)
    //     {
    //         std::shared_ptr<open3d::geometry::Segment3D> segm = std::make_shared<open3d::geometry::Segment3D>(pts[i], pts[(i+1)%pts.size()]);
    //         std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
    //         segLineset->points_.push_back(segm->Origin());
    //         segLineset->points_.push_back(segm->EndPoint());
    //         segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
    //         segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //         segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //         segLineset->PaintUniformColor(Eigen::Vector3d(0., 0., 1.));
    //         vis->AddGeometry(segLineset);
    //     }
    // }

    // // show just one polygon
    // std::vector<Eigen::Vector3d> pts = splitPolygons[0].getPoints();
    // for (int i = 0; i < pts.size(); i++)
    // {
    //     std::shared_ptr<open3d::geometry::Segment3D> segm = std::make_shared<open3d::geometry::Segment3D>(pts[i], pts[(i+1)%pts.size()]);
    //     std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
    //     segLineset->points_.push_back(segm->Origin());
    //     segLineset->points_.push_back(segm->EndPoint());
    //     segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
    //     segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //     segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //     segLineset->PaintUniformColor(Eigen::Vector3d(0., 1., 1.));
    //     vis->AddGeometry(segLineset);
    // }



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
            // b. save the result into a polygon
            planeIntersections->reserve(outPtsCount);
            planeIntersections->clear();
            for (unsigned int i = 0; i < outPtsCount; i++)
            {
                planeIntersections->push_back(outPtsPtr[i]);
            }

            TSPolygon tempPoly = TSPolygon(*planeIntersections, t.getPlane());
            tempPoly.reorderClockwisePoints();
            this->m_PlnAABBPolygons.push_back(tempPoly);
        }
        delete outPtsPtr;
        delete planeIntersections;


        this->timeEnd();
    }
    bool TSGeometricSolver::rRay2PlaneIntersection(const Eigen::Vector3d &RayOrig,
                                                   const Eigen::Vector3d &RayDir,
                                                   const TSPlane &Plane,
                                                   float *OutT,
                                                   float *OutVD)
    {
        const Eigen::Vector3d PlaneNormal(Plane.A, Plane.B, Plane.C);

        const double Denominator = PlaneNormal.dot(RayDir);
        if (Denominator == 0.0f)  return false;  // ray is parallel to plane

        const double Numerator = Plane.D - PlaneNormal.dot(RayOrig);
        const double t = Numerator / Denominator;

        if (OutT) *OutT = t;
        if (OutVD) *OutVD = Denominator;

        return true;
    }
    void TSGeometricSolver::rPlane2AABBSegmentIntersect(const TSPlane &plane,
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
        if (plane.A * orig.x() + plane.B * orig.y() + plane.C * orig.z() + plane.D == 0.f)
            out_points[out_point_count++] = orig;
    }

    void TSGeometricSolver::rIntersectMeanPolygonPlnAABB()
    {
        this->rMeanPolygonPlanes();

        Eigen::Vector3d* outPtsPtr2 = new Eigen::Vector3d[3*6];
        for (auto& mpoly : this->m_MergedPolygons)
        {
            // a. caculate the intersection points
            unsigned int outPtsCount2;
            this->rPlane2AABBSegmentIntersect(mpoly.getLinkedPlane(),
                                              this->m_Timber->getAABB().min_bound_, 
                                              this->m_Timber->getAABB().max_bound_,
                                              outPtsPtr2,
                                              outPtsCount2);

            // b. store the new itnersection points into the polygon
            for (unsigned int i = 0; i < outPtsCount2; i++)
            {
                mpoly.addPoint(outPtsPtr2[i]);
            }
            mpoly.reorderClockwisePoints();
        }
        delete outPtsPtr2;
    }
    void TSGeometricSolver::rMeanPolygonPlanes()
    {
        open3d::geometry::KDTreeFlann kdtree;

        std::shared_ptr<open3d::geometry::PointCloud> pntCldPCtr = std::make_shared<open3d::geometry::PointCloud>();
        for (auto& p : this->m_PlnAABBPolygons) pntCldPCtr->points_.push_back(p.getCenter());
        kdtree.SetGeometry(*pntCldPCtr);

        std::vector<int> indices;
        std::vector<int> mergedIndices;
        std::vector<double> distances;

        Eigen::Vector3d meanPt;
        Eigen::Vector3d meanNorm;

        for (int i=0; i < pntCldPCtr->points_.size(); i++)
        {
            kdtree.SearchRadius(pntCldPCtr->points_[i], this->m_MinPolyDist, indices,distances);

            int k = 0;
            meanPt = {0,0,0};
            meanNorm = {0,0,0};
            
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
            }

            if (k > 1)
            {
                meanPt /= k;
                meanNorm /= k;
                TSPolygon newPoly;
                newPoly.setLinkedPlane(TSPlane(meanNorm, meanPt));
                this->m_MergedPolygons.push_back(newPoly);
            }
        }
    }

    void TSGeometricSolver::rIntersectPolygons(std::vector<TSPolygon> &polygons,
                                               std::vector<TSSegment> &segments)
    {
        for (auto& polyA : this->m_MergedPolygons)
        {
            for (auto& polyB : this->m_MergedPolygons)
            {
                TSSegment seg;
                bool isIntersect = false;
                isIntersect = polyA.intersectPolygon(polyB, seg);

                // check the segments are unique
                for (auto& s : segments)
                {
                    if (s.P1.isApprox(seg.P1) && s.P2.isApprox(seg.P2)
                        || s.P1.isApprox(seg.P2) && s.P2.isApprox(seg.P1))
                    {
                        isIntersect = false;
                        break;
                    }
                }
                if (isIntersect) segments.push_back(seg);
            }
        }
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