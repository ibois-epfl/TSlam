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
        this->rIntersectMeanPolygonPlnAABB();


        // // here we can split the polygons in two (clockwise and counter clockwise)
        // // we can do this by checking the distance of the intersection points to the center of the polygon
        // // the ones closer to the center are the ones in the clockwise direction
        // // the ones further away are the ones in the counter clockwise direction
        // // we can then create two new polygons and add them to the list of polygons ?
        // // we can then remove the old polygon from the list of polygons ?

        // std::vector<Eigen::Vector3d> clockwisePts;
        // std::vector<Eigen::Vector3d> counterClockwisePts;




        std::shared_ptr<open3d::geometry::PointCloud> pntCldIntersect = std::make_shared<open3d::geometry::PointCloud>();
        std::vector<TSPolygon> splitPolygonsA;
        std::vector<TSPolygon> splitPolygonsB;

        
        bool isIntersect = false;
        bool isAlreadyIn = true;

        std::vector<Eigen::Vector3d> tempIntersectPts;
        Eigen::Vector3d* intersectPt = new Eigen::Vector3d();

        int KK = 0;  // TODO: debug erase

        for (auto& polyA : this->m_MergedPolygons)
        {
            for (auto& polyB : this->m_MergedPolygons)
            {
                if (polyA != polyB)
                {
                    // std::tuple<TSPolygon, TSPolygon> splitPolysTest;                            // TODO: debug test
                    // bool test = this->rPolygon2PolygonIntersect(polyA, polyB, splitPolysTest);  // TODO: debug test
                    // if (test)
                    // {
                    //     splitPolygonsA.push_back(std::get<0>(splitPolysTest));
                    //     splitPolygonsB.push_back(std::get<1>(splitPolysTest));
                    // }

                    for (int i = 0; i < polyA.size(); i++)
                    {
                        for (int j = 0; j < polyB.size(); j++)
                        {
                            isIntersect = this->rSegment2SegmentIntersect(polyA[i], polyB[j], intersectPt);

                            isAlreadyIn = false;
                            for (auto& pt : pntCldIntersect->points_)
                            {
                                if (pt.isApprox(*intersectPt, 1e-6))
                                {
                                    isAlreadyIn = true;
                                    break;
                                }
                            }
                            if (isIntersect && !isAlreadyIn)
                            {
                                tempIntersectPts.push_back(*intersectPt);
                                pntCldIntersect->points_.push_back(*intersectPt);
                            }
                        }
                    }

                    // // // =============================================================================
                    
                    // std::cout << "Length of tempIntersectPts: " << tempIntersectPts.size() << std::endl;
                    if (tempIntersectPts.size() == 2)
                    {
                        // KK++;
                        // if (KK == 10) // TODO: debug erase // not working indexes: 5
                        // {
                            // the intersection points are always 2 per polygon

                            TSSegment segSplit(tempIntersectPts[0], tempIntersectPts[1]);

                            std::tuple<TSPolygon, TSPolygon> splitPolys = polyA.splitPolygon(segSplit);
                            splitPolygonsA.push_back(std::get<0>(splitPolys));
                            splitPolygonsB.push_back(std::get<1>(splitPolys));

                        //     goto endLoop;  // TODO: debug erase
                        // }

                    }


                    // clear out the intersect points vector
                    tempIntersectPts.clear();

                    // // =============================================================================

                }
            }
        }
        // endLoop:  // TODO: debug erase
        delete intersectPt;

        

        std::cout << "Intersect points: " << pntCldIntersect->points_.size() << std::endl;


        // // =============================================================================
        // now we take the intersections when detected and we recreate two polygons in clockwise and counter clockwise order



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

    // // draw polygon segments3D 
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

    // // draw new interesected polygons
    // for (auto& pg : this->m_MergedPolygons)
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

    // show polygons intersect points
    pntCldIntersect->PaintUniformColor(Eigen::Vector3d(1, 0, 1));
    vis->AddGeometry(pntCldIntersect);

    // // debug: show selected polygons polyA and polyB
    // std::vector<TSSegment> polyASegs = polyA.getSegments();
    // for (auto& seg : polyASegs)
    // {
    //     std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
    //     segLineset->points_.push_back(seg.Origin());
    //     segLineset->points_.push_back(seg.EndPoint());
    //     segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
    //     segLineset->colors_.push_back(Eigen::Vector3d(1, 0, 0));
    //     segLineset->colors_.push_back(Eigen::Vector3d(1, 0, 0));
    //     segLineset->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
    //     vis->AddGeometry(segLineset);
    // }
        
    // std::vector<TSSegment> polyBSegs = polyB.getSegments();
    // for (int i = 0; i < polyBSegs.size(); i++)
    // {
    //     std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
    //     segLineset->points_.push_back(polyBSegs[i].Origin());
    //     segLineset->points_.push_back(polyBSegs[i].EndPoint());
    //     segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
    //     segLineset->colors_.push_back(Eigen::Vector3d(1, 0, 0));
    //     segLineset->colors_.push_back(Eigen::Vector3d(1, 0, 0));
    //     segLineset->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
    //     vis->AddGeometry(segLineset);
    // }

    // // debug: show only two segments
    // std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
    // segLineset->points_.push_back(segA.Origin());
    // segLineset->points_.push_back(segA.EndPoint());
    // segLineset->points_.push_back(segB.Origin());
    // segLineset->points_.push_back(segB.EndPoint());
    // segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
    // segLineset->lines_.push_back(Eigen::Vector2i(2, 3));
    // segLineset->colors_.push_back(Eigen::Vector3d(1, 0, 0));
    // segLineset->colors_.push_back(Eigen::Vector3d(1, 0, 0));
    // segLineset->colors_.push_back(Eigen::Vector3d(1, 0, 0));
    // segLineset->colors_.push_back(Eigen::Vector3d(1, 0, 0));
    // segLineset->PaintUniformColor(Eigen::Vector3d(1, 1, 0));
    // vis->AddGeometry(segLineset);

    // show split polygons
    for (auto& pg : splitPolygonsA)
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
            segLineset->PaintUniformColor(Eigen::Vector3d(1., 0., 1.));
            vis->AddGeometry(segLineset);
        }
    }
    // for (auto& pg : splitPolygonsB)
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
    //         segLineset->PaintUniformColor(Eigen::Vector3d(0., 1., 1.));
    //         vis->AddGeometry(segLineset);
    //     }
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

    void TSGeometricSolver::rIntersectPolygons()
    {
        // TODO
    }
    bool TSGeometricSolver::rSegment2SegmentIntersect(TSSegment& seg1, 
                                                      TSSegment& seg2, 
                                                      Eigen::Vector3d* intersectPt)
    {
        Eigen::Vector3d A = seg1.Origin();
        Eigen::Vector3d B = seg1.EndPoint();
        Eigen::Vector3d C = seg2.Origin();
        Eigen::Vector3d D = seg2.EndPoint();

        Eigen::Vector3d AB = B - A;
        Eigen::Vector3d CD = D - C;

        Eigen::Vector3d cross = AB.cross(CD);
        if (cross.norm() == 0.f) return false;

        Eigen::Vector3d AC = C - A;

        double t = AC.cross(CD).dot(cross) / cross.squaredNorm();
        double u = AC.cross(AB).dot(cross) / cross.squaredNorm();

        if (t >= 0.f && t <= 1.f && u >= 0.f && u <= 1.f)
        {
            Eigen::Vector3d ptTemp = A + t * AB;

            if (seg1.isPointOnSegment(ptTemp) && seg2.isPointOnSegment(ptTemp))
            {
                *intersectPt = ptTemp;
                return true;
            }
            else return false;
        }
        else return false;
    }
    bool TSGeometricSolver::rPolygon2PolygonIntersect(TSPolygon& polyA, 
                                                      TSPolygon& polyB,
                                                      std::tuple<TSPolygon, TSPolygon>& outSplitPolygons)
    {
        if (polyA == polyB) return false;

        TSSegment segSplit;
        uint idx = 0;
        Eigen::Vector3d* intersectPt = new Eigen::Vector3d();
        bool isIntersect = false;

        for (int i = 0; i < polyA.size(); i++)
        {
            for (int j = 0; j < polyB.size(); j++)
            {
                isIntersect = false;
                if (polyA[i] == polyB[j]) continue;

                isIntersect = this->rSegment2SegmentIntersect(polyA[i], polyB[j], intersectPt);

                if (isIntersect && idx <= 2)
                {
                    segSplit.P1 = polyA[i].Origin();
                    segSplit.P2 = polyA[i].EndPoint();
                    idx++;
                }
                else if (isIntersect && idx > 2)
                {
                    std::cout << "[ERROR]: degenerated intersection: more than 2 points." << std::endl;
                    return false;
                }
            }
        }

        polyA.reorderClockwisePoints();
        outSplitPolygons = polyA.splitPolygon(segSplit);

        delete intersectPt;

        return true;

        // // =============================================================================
        
        // if (tempIntersectPts.size() != 2) return false;
         
        // TSSegment segSplit(tempIntersectPts[0], tempIntersectPts[1]);
        outSplitPolygons = polyA.splitPolygon(segSplit);

        // clear out the intersect points vector
        // tempIntersectPts.clear();

        // // =============================================================================

        
        return true;
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