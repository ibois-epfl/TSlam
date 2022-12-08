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




        std::shared_ptr<open3d::geometry::PointCloud> pntCldIntersect = std::make_shared<open3d::geometry::PointCloud>();
        bool isIntersect = false;
        Eigen::Vector3d* intersectPt = new Eigen::Vector3d();
        // test each polygon against all the others for intersection
        for (auto& pg : this->m_MergedPolygons)  /// Polygons
        {
            for (auto& qg : this->m_MergedPolygons)   /// Polygons
            {
                if (pg != qg)
                {
                    for (int i = 0; i < pg.size(); i++)  /// Segments
                    {
                        for (int j = 0; j < qg.size(); j++)  /// Segments
                        {
                            isIntersect = this->rSegment2SegmentIntersect(pg[i].Origin(), pg[i].EndPoint(), 
                                                                          qg[j].Origin(), qg[j].EndPoint(),
                                                                          intersectPt);
                            if (isIntersect)
                            {
                                pntCldIntersect->points_.push_back(*intersectPt);
                                // break;
                            }
                        }
                        // if (isIntersect)
                            // break;
                    }
                    if (isIntersect)  /// ?
                    {
                        std::cout << "Intersecting polygons found!" << std::endl;
                        break;
                    }
                }
            }
            // if (isIntersect)  /// ?
            //     break;
        }

        std::cout << "Number of intersecting points: " << pntCldIntersect->points_.size() << std::endl;



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

    // show polygons intersect points
    pntCldIntersect->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
    vis->AddGeometry(pntCldIntersect);

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
                                                   const TSPlane &Plane,
                                                   float *OutT,
                                                   float *OutVD)
    {
        const Eigen::Vector3d PlaneNormal(Plane.A, Plane.B, Plane.C);  /// FIXME: change to Getters from TSPlane

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
    void TSGeometricSolver::rSortIntersectionPoints(Eigen::Vector3d* points,
                                                    unsigned point_count,
                                                    const TSPlane& plane)
    {
        Eigen::Vector3d center = Eigen::Vector3d(0.f, 0.f, 0.f);

        // compute the center of the polygon
        for  (unsigned i = 0; i < point_count; ++i)
            center += points[i];
        center /= (float)point_count;

        // sort the points in a clockwise order also for the case of plane.A < 0.f
        std::sort(points, points + point_count, [center, plane](const Eigen::Vector3d& a, const Eigen::Vector3d& b)
        {
            // compute the angle between the center and the two points
            float angleA = atan2(a.y() - center.y(), a.x() - center.x());
            float angleB = atan2(b.y() - center.y(), b.x() - center.x());

            // sort the points by the angle
            if (angleA < angleB)
                return true;
            else if (angleA > angleB)
                return false;
            else
            {
                // if the angle is the same, sort the points by the distance
                float distA = (a - center).norm();
                float distB = (b - center).norm();
                return distA < distB;
            }
        });

        // compute the normal of the polygon
        Eigen::Vector3d normal = Eigen::Vector3d(plane.A, plane.B, plane.C);
        if (normal.dot(center) < 0.f)
            normal *= -1.f;

        // check if the points are in the right order
        Eigen::Vector3d ab = points[1] - points[0];
        Eigen::Vector3d bc = points[2] - points[1];
        Eigen::Vector3d ca = points[0] - points[2];
        Eigen::Vector3d n = ab.cross(bc);
        if (n.dot(normal) < 0.f)
        {
            // reverse the order of the points
            for (unsigned i = 0; i < point_count / 2; ++i)
            {
                Eigen::Vector3d tmp = points[i];
                points[i] = points[point_count - i - 1];
                points[point_count - i - 1] = tmp;
            }
        }
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
            this->rSortIntersectionPoints(outPtsPtr2, outPtsCount2, mpoly.getLinkedPlane());

            // b. store the new itnersection points into the polygon
            for (unsigned int i = 0; i < outPtsCount2; i++)
            {
                mpoly.addPoint(outPtsPtr2[i]);
            }
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
    bool TSGeometricSolver::rSegment2SegmentIntersect(const Eigen::Vector3d& p1, 
                                                      const Eigen::Vector3d& p2, 
                                                      const Eigen::Vector3d& p3, 
                                                      const Eigen::Vector3d& p4,
                                                      Eigen::Vector3d* intersectPt)
    {
        /*
        (a1,a2,a3)________________(b1,b2,b3)
        A                           B 

        (c1,c2,c3)________________(d1,d2,d3)
        C                            D
        */
        double A = p2.x() - p1.x();
        double B = p3.x() - p4.x();
        double C = p3.x() - p1.x();
        double D = p2.y() - p1.y();
        double E = p3.y() - p4.y();
        double F = p3.y() - p1.y();

        // find t using formula t=(C*E-F*B)/(E*A-B*D)
        // and s=(C*D-A*F)/(E*A-B*D)
        double t = (C*E-F*B)/(E*A-B*D);
        double s = (C*D-A*F)/(E*A-B*D);

        // check if t and s are between 0 and 1
        if ((t*(1-t) >= 0) && (s*(1-s) >= 0))
        {
            // find the intersection point
            *intersectPt = p1 + t*(p2-p1);
            return true;
        }
        else
            return false;


        // // ===================================================================================

        // check if the segments are parallel
        // Eigen::Vector3d v1 = p2 - p1;
        // Eigen::Vector3d v2 = p4 - p3;
        // Eigen::Vector3d v3 = p3 - p1;

        // float dot = v1.x() * v2.y() - v1.y() * v2.x();
        // if (dot == 0.f)
        //     return false;
        
        // // find the intersection point
        // float t = (v1.x() * v3.y() - v1.y() * v3.x()) / dot;
        // float u = (v2.x() * v3.y() - v2.y() * v3.x()) / dot;

        // if (t >= 0.f && t <= 1.f && u >= 0.f && u <= 1.f)
        // {
        //     *intersectPt = p1 + t * v1;
        //     return true;
        // }
        // else
        //     return false;


        // // ===================================================================================

        // // find 3dimensional intersection (x,y,z) of two 3d segments
        // // Line AB represented as a1x + b1y = c1
        // double a1 = p2.y() - p1.y();
        // double b1 = p1.x() - p2.x();
        // double c1 = a1*(p1.x()) + b1*(p1.y());

        // // Line CD represented as a2x + b2y = c2
        // double a2 = p4.y() - p3.y();
        // double b2 = p3.x() - p4.x();
        // double c2 = a2*(p3.x())+ b2*(p3.y());

        // // Line z represented as a3x + b3y = c3
        // double a3 = p2.z() - p1.z();
        // double b3 = p1.x() - p2.x();
        // double c3 = a3*(p1.x()) + b3*(p1.y());

        // double determinant = a1*b2 - a2*b1;

        // // check if the lines are parallel
        // if (determinant == 0) return false;

        // double x = (b2*c1 - b1*c2)/determinant;
        // double y = (a1*c2 - a2*c1)/determinant;
        // double z = (a3*c1 - a1*c3)/determinant;

        // *intersectPt = Eigen::Vector3d(x,y,z);

        // // check if the intersection point is on both segments TODO: check if correct
        // TSSegment seg1(p1, p2);  /// TODO: replace in function param
        // TSSegment seg2(p3, p4);  /// TODO: replace in function param
        // if (seg1.isPointOnSegment(*intersectPt) && seg2.isPointOnSegment(*intersectPt))
        //     return true;
        // else
        //     return false;

        // // ===================================================================================

        // // Line AB represented as a1x + b1y = c1
        // double a1 = p2.y() - p1.y();
        // double b1 = p1.x() - p2.x();
        // double c1 = a1*(p1.x()) + b1*(p1.y());

        // // Line CD represented as a2x + b2y = c2
        // double a2 = p4.y() - p3.y();
        // double b2 = p3.x() - p4.x();
        // double c2 = a2*(p3.x())+ b2*(p3.y());

        // // Line z represented as a3x + b3y = c3

        // double determinant = a1*b2 - a2*b1;

        // // check if the lines are parallel
        // if (determinant == 0) return false;

        // double x = (b2*c1 - b1*c2)/determinant;
        // double y = (a1*c2 - a2*c1)/determinant;
        // *intersectPt = {x,y,0};
        // return true;
    }
    void TSGeometricSolver::rPoly2PolyIntersect(const TSPolygon& poly1, 
                                               const TSPolygon& poly2,
                                               Eigen::Vector3d* out_points, 
                                               unsigned& out_point_count)
    {
        // intersect two polygons and create new polygons out of the intersection
        // check if the polygons are intersecting
        
        
    }
    void rPointInPoly(const Eigen::Vector3d& point, const TSPolygon& poly)
    {
        //TODO;
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