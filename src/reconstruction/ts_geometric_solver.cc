#include "ts_geometric_solver.hh"
#include<open3d/Open3D.h>
#include <stdexcept>
#include <algorithm>
#include <math.h>
#include <map>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <random>
#include <array>


namespace tslam
{
    void TSGeometricSolver::reconstruct()
    {
        this->rDetectFacesStripes();
        this->rIntersectStripeTagPlnAABB();
        this->rCreatePolysurface();
        // this->rCreateMesh();

        this->visualize(this->m_ShowVisualizer,
                        this->m_DrawTags,
                        this->m_DrawTagTypes,
                        this->m_DrawTagNormals,
                        this->m_DrawAabb,
                        this->m_DrawIntersectedPoly,
                        this->m_DrawSplittingSegments,
                        this->m_DrawSelectedFace,
                        this->m_DrawFinalMesh);
        
        // this->exportMesh2PLY(this->m_DirOut, this->m_FilenameOut);  // FIXME: this might not be 
    }

    void TSGeometricSolver::visualize(bool showVisualizer,
                                      bool drawTags,
                                      bool drawTagTypes,
                                      bool drawTagNormals,
                                      bool drawAabb,
                                      bool drawIntersectedPoly,
                                      bool drawSplittingSegments,
                                      bool drawSelectedFaces,
                                      bool drawFinalMesh)
    {
#ifdef TSLAM_REC_DEBUG
        if (!showVisualizer)
            return;
        open3d::visualization::Visualizer* vis(new open3d::visualization::Visualizer());
        vis->CreateVisualizerWindow("TSPlaneTags", 1920, 1080);

        if (drawTags)
        {
            std::vector<Eigen::Vector3d> clrs;
            for (int i = 0; i < 10; i++)
            {
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_real_distribution<> dis(0, 1);
                Eigen::Vector3d color = Eigen::Vector3d(dis(gen), dis(gen), dis(gen));
                clrs.push_back(color);
            }
            for (auto& tag : this->m_Timber->getPlaneTags())
            {
                open3d::geometry::TriangleMesh tagBase = tag.getOpen3dMesh();
                auto planeTagsLineset1 = open3d::geometry::LineSet::CreateFromTriangleMesh(tagBase);
                planeTagsLineset1->PaintUniformColor(clrs[tag.getFaceIdx()]);

                vis->AddGeometry(planeTagsLineset1);
            }
        }

        if (drawTagTypes)
        {
            std::shared_ptr<open3d::geometry::PointCloud> tagCenters = std::make_shared<open3d::geometry::PointCloud>();
            for (auto& tag : this->m_Timber->getPlaneTags())
            {
                if (tag.isEdge())
                {
                    tagCenters->points_.push_back(tag.getCenter());
                    tagCenters->colors_.push_back(Eigen::Vector3d(0, 1, 0));
                }
            }
            vis->AddGeometry(tagCenters);
        }

        if (drawTagNormals)
        {
            std::shared_ptr<open3d::geometry::LineSet> tagNormals = std::make_shared<open3d::geometry::LineSet>();
            for (auto& tag : this->m_Timber->getPlaneTags())
            {
                tagNormals->points_.push_back(tag.getCenter());
                tagNormals->points_.push_back(tag.getCenter() + tag.getNormal() * 1.0);
                tagNormals->lines_.push_back(Eigen::Vector2i(tagNormals->points_.size() - 2, tagNormals->points_.size() - 1));
            }
            vis->AddGeometry(tagNormals);
        }
        
        if (drawAabb)
            std::shared_ptr<open3d::geometry::LineSet> aabbLineset = std::make_shared<open3d::geometry::LineSet>();

        if (drawIntersectedPoly)
        {
            for (auto& pg : this->m_PlnAABBPolygons)
            {
                std::vector<Eigen::Vector3d> pts = pg.getVertices();
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
        }

        if (drawSplittingSegments)
        {
            // TODO: clean the visualizer up for splitting segments
            // for (auto& segm : this->m_SplitSegments)
            // {
            //     std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
            //     segLineset->points_.push_back(segm.P1);
            //     segLineset->points_.push_back(segm.P2);
            //     segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
            //     segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
            //     segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
            //     segLineset->PaintUniformColor(Eigen::Vector3d(1., 0., 1));
            //     vis->AddGeometry(segLineset);
            // }

            for (auto& segmGroup : this->m_SplitSegmentsGrouped)
            {
                std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
                for (auto& segm : segmGroup)
                {
                    segLineset->points_.push_back(segm.P1);
                    segLineset->points_.push_back(segm.P2);
                    segLineset->lines_.push_back(Eigen::Vector2i(segLineset->points_.size() - 2, segLineset->points_.size() - 1));
                    segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
                    segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
                }
                segLineset->PaintUniformColor(Eigen::Vector3d(1., 0., 0));
                vis->AddGeometry(segLineset);
            }
        }

        if (drawSelectedFaces)
        {
            for (auto& pg : this->m_FacePolygons)
            {
                std::vector<Eigen::Vector3d> pts = pg.getVertices();
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
        }
        
        if (drawFinalMesh)
        {
            std::shared_ptr<open3d::geometry::TriangleMesh> mesh = std::make_shared<open3d::geometry::TriangleMesh>(this->m_MeshOut);
            mesh->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
            vis->AddGeometry(mesh);

            std::shared_ptr<open3d::geometry::PointCloud> pcdMeshCenters = std::make_shared<open3d::geometry::PointCloud>();
            for (auto& tri : mesh->triangles_)
            {
                Eigen::Vector3d ctr = (mesh->vertices_[tri(0)] + mesh->vertices_[tri(1)] + mesh->vertices_[tri(2)]) / 3.;
                pcdMeshCenters->points_.push_back(ctr);
            }
            pcdMeshCenters->PaintUniformColor(Eigen::Vector3d(1., 0., 0.));
            vis->AddGeometry(pcdMeshCenters);
        }

        vis->Run();
        vis->Close();
        vis->DestroyVisualizerWindow();
#endif
    }


    void TSGeometricSolver::rDetectFacesStripes()
    {
        std::vector<std::shared_ptr<TSRTStripe>> tempStripes;

        this->rParseTags2Stripes(tempStripes);
        this->rRefineStripesByPlanes(tempStripes);

        this->m_Timber->setTSRTagsStripes(tempStripes);
    }
    void TSGeometricSolver::rParseTags2Stripes(std::vector<std::shared_ptr<TSRTStripe>>& stripes)
    {
        // copy tags for destructive operations
        std::unique_ptr<open3d::geometry::PointCloud> ctrsCopyPtr = 
            std::make_unique<open3d::geometry::PointCloud>(this->m_Timber->getTagsCtrs());
        auto tagsCopy = this->m_Timber->getPlaneTags();

        // output stripes
        std::shared_ptr<TSRTStripe> stripe = std::make_shared<TSRTStripe>();

        // properties for kdtree
        open3d::geometry::KDTreeFlann kdtree;
        kdtree.SetGeometry(*ctrsCopyPtr);
        std::vector<int> indices;
        std::vector<double> distances;
        uint knn = 2;
        uint faceIdx = 0;
        uint idx = 0;
        uint nextIdx = 0;

        do
        {
            // refresh the new kdtree
            kdtree.SetGeometry(*ctrsCopyPtr);

            // find the nearest neighborh
            kdtree.SearchKNN(ctrsCopyPtr->points_[idx], knn, indices, distances);
            nextIdx = indices[1];

            // compute the angle between the two tags
            double angle = tslam::TSVector::angleBetweenVectors(
                tagsCopy[idx].getNormal(),
                tagsCopy[nextIdx].getNormal());
            
            // (a) face: if the angle is below the threshold
            if (angle < this->m_CreaseAngleThreshold)
            {
                tagsCopy[idx].setFaceIdx(faceIdx);
                stripe->push_back(tagsCopy[idx]);
            }
            // (b) crease: if the angle is above the threshold
            else if (angle > this->m_CreaseAngleThreshold)
            {
                tagsCopy[idx].setFaceIdx(faceIdx);
                faceIdx++;
                tagsCopy[nextIdx].setFaceIdx(faceIdx);

                stripe->push_back(tagsCopy[idx]);
                
                std::shared_ptr<TSRTStripe> stripeCopy = std::make_shared<TSRTStripe>(*stripe);
                stripes.push_back(stripeCopy);

                stripe->clear();
                stripe->push_back(tagsCopy[nextIdx]);
            }

            // get rid of the current tag
            tagsCopy.erase(tagsCopy.begin() + idx);
            ctrsCopyPtr->points_.erase(ctrsCopyPtr->points_.begin() + idx);

            // set the next index as the current index
            idx = (nextIdx > idx) ? nextIdx - 1 : nextIdx;

        } while(ctrsCopyPtr->points_.size() > 0);

        // add the last stripe
        stripes.push_back(stripe);

        // reorder the tags on each stripe
        for (auto& stripe : stripes)
            stripe->reorderTags();
    }
    void TSGeometricSolver::rRefineStripesByPlanes(std::vector<std::shared_ptr<TSRTStripe>>& stripes)
    {
        for (int i = 0; i < stripes.size(); i++)
        {
            for (int j = i + 1; j < stripes.size(); j++)
            {
                double dist = stripes[i]->getMeanPlane().distance(stripes[j]->getMeanPlane());
                double angle = stripes[i]->getMeanPlane().Normal.dot(stripes[j]->getMeanPlane().Normal);
                double angleDeg = acos(angle) * 180. / M_PI;

                if (stripes[i]->getMeanPlane().distance(stripes[j]->getMeanPlane()) < this->m_MaxPlnDist2Merge &&
                    (angleDeg < this->m_MaxPlnAngle2Merge || angleDeg > 180 - this->m_MaxPlnAngle2Merge)
                    )
                {
                    *stripes[i] += *stripes[j];

                    TSPlane avgPlane = TSPlane(stripes[i]->getNormal(), 
                                                stripes[i]->front().getCenter(), 
                                                stripes[i]->back().getCenter());

                    stripes[i]->setMeanPlane(avgPlane);

                    stripes.erase(stripes.begin() + j);
                    j--;
                }
            }
        }
    }

    void TSGeometricSolver::rIntersectStripeTagPlnAABB()
    {
        // scale the AABB
        this->m_Timber->scaleAABB(this->m_AABBScaleFactor);

        // intersect the planes with the timber's AABB
        Eigen::Vector3d* outPtsPtr = new Eigen::Vector3d[3*6];
        std::vector<Eigen::Vector3d>* planeIntersections = new std::vector<Eigen::Vector3d>();

        for (int i = 0; i < this->m_Timber->getTSRTagsStripes().size(); i++)
        {
            // a. intersect the plane with the AABB
            unsigned int outPtsCount;
            tslam::TSPlane::plane2AABBSegmentIntersect(this->m_Timber->getTSRTagsStripes()[i]->getMeanPlane(),
                                                       this->m_Timber->getAABB().min_bound_,
                                                       this->m_Timber->getAABB().max_bound_,
                                                       outPtsPtr,
                                                       outPtsCount);
            // b. save the result into a polygon
            planeIntersections->reserve(outPtsCount);
            planeIntersections->clear();
            for (unsigned int i = 0; i < outPtsCount; i++)
                planeIntersections->push_back(outPtsPtr[i]);

            // if (i==2)  // DEBUG  i.e. 4 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            // {
                TSPolygon tempPoly = TSPolygon(*planeIntersections, this->m_Timber->getTSRTagsStripes()[i]->getMeanPlane());
                tempPoly.reorderClockwisePoints();
                this->m_PlnAABBPolygons.push_back(tempPoly);
            // }
        }
        delete outPtsPtr;
        delete planeIntersections;

        // delete duplicates from the polygons list (e.g. double stripes on the same plane for synthetic data)
        for (int i = 0; i < this->m_PlnAABBPolygons.size(); i++)
        {
            for (int j = i + 1; j < this->m_PlnAABBPolygons.size(); j++)
            {
                if (this->m_PlnAABBPolygons[i] == this->m_PlnAABBPolygons[j])
                {
                    this->m_PlnAABBPolygons.erase(this->m_PlnAABBPolygons.begin() + j);
                    j--;
                }
            }
        }
    }

    void TSGeometricSolver::rCreatePolysurface()
    {
        this->rIntersectPolygons(this->m_PlnAABBPolygons, this->m_SplitSegmentsGrouped);


        this->rIntersectSplittingSegments(this->m_SplitSegmentsGrouped, this->m_SplitPolygons);

        this->rSelectFacePolygons(this->m_SplitPolygons,
                                  this->m_FacePolygons,
                                  this->m_MaxPolyTagDist);
    }
    void TSGeometricSolver::rIntersectPolygons(std::vector<TSPolygon> &polygons,
                                               std::vector<std::vector<TSSegment>> &segmentsGrouped)
    {
        // (a) intersect polygons among themselves to find the splitting segments' extremes
        std::vector<TSSegment> tempSegments;
        for (auto& polyA : polygons)
        {
            for (auto& polyB : polygons)
            {
                TSSegment seg;
                bool isIntersect = false;
                isIntersect = polyA.intersectPolygon(polyB, seg);

                // check the segments are unique
                for (auto& s : tempSegments)
                {
                    if (s == seg)
                    {
                        isIntersect = false;
                        break;
                    }
                }
                if (isIntersect) tempSegments.push_back(seg);
            }

            if (tempSegments.size() > 0)
            {
                segmentsGrouped.push_back(tempSegments);
                tempSegments.clear();
            }
        }

        // (*b) intersect the segments among themselves and if they have only one intersection point discard them
        // auto segmentsGroupedTemp = segmentsGrouped;
        for (auto& segGroup : segmentsGrouped)
        {
            for (uint i = 0; i < segGroup.size(); i++)
            {
                std::vector<Eigen::Vector3d> intersectionPts;
                for (auto& segB : segGroup)
                {
                    if (segGroup[i] == segB) continue;

                    Eigen::Vector3d intersectionPt;
                    bool isIntersect = false;
                    isIntersect = segGroup[i].intersect(segB, intersectionPt);


                    if (isIntersect)
                    {
                        intersectionPts.push_back(intersectionPt);
                    }
                }
                if (intersectionPts.size() == 1)
                {
                    // discard the segment
                    for (auto& seg : segGroup)
                    {
                        if (seg == segGroup[i])
                        {
                            segGroup.erase(segGroup.begin() + i);
                            break;
                        }
                    }
                }
            }
        }
    }
    void TSGeometricSolver::rIntersectSplittingSegments(std::vector<std::vector<TSSegment>> &segmentsGrouped,
                                                        std::vector<TSPolygon>& polygons)
    {
        std::cout << "<<<<<<<<<< rIntersectSplittingSegments >>>>>>>>>>" << std::endl;  // DEBUG
        // TODO: find a way to obtain all minimum area polygons from the segments's intersections

        // store all the segments in a single vector
        std::vector<TSSegment> allSegments;
        for (auto& segGroup : segmentsGrouped)
            for (auto& seg : segGroup)
                allSegments.push_back(seg);
        
        std::vector<Eigen::Vector3d> intersectionPts;

        //////////////////// ATTEMPT 2 ////////////////////


        // find all the intersection points
        std::vector<Eigen::Vector3d> intersectionPts;

        for (int i = 0; i < segmentsGrouped.size(); i++)
        {
            for (auto& segA : segmentsGrouped[i])
            {
                for (auto& segB : allSegments)
                {
                    if (segA == segB) continue;

                    bool isIntersect = false;
                    Eigen::Vector3d intersectionPt;
                    isIntersect = segA.intersect(segB, intersectionPt);

                    if (isIntersect)
                    {
                        // check the intersection point is unique
                        bool isUnique = true;
                        for (auto& pt : intersectionPts)
                        {
                            if (pt.isApprox(intersectionPt, 1e-6))
                            {
                                isUnique = false;
                                break;
                            }
                        }
                        if (isUnique) intersectionPts.push_back(intersectionPt);
                    }
                }
            }

            std::cout << "intersectionPts.size() = " << intersectionPts.size() << std::endl;  // DEBUG
            std::cout << "--------------------------------------------------" << std::endl;  // DEBUG

            // create a polygon from the intersection points
            if (intersectionPts.size() > 0)
            {
                // subdivide the larger polygons into smaller ones 
                if (intersectionPts.size() >= 4)
                {
                    // check if the number of intersection points is even
                    if (intersectionPts.size() % 2 != 0)
                    {
                        std::cout << "[ERROR]: the number of intersection points is not even" << std::endl;
                        return;
                    }

                    // find the two points that are the farthest from each other
                    double maxDist = 0;
                    Eigen::Vector3d ptA, ptB;
                    for (int i = 0; i < intersectionPts.size(); i++)
                    {
                        for (int j = i + 1; j < intersectionPts.size(); j++)
                        {
                            double dist = (intersectionPts[i] - intersectionPts[j]).norm();
                            if (dist > maxDist)
                            {
                                maxDist = dist;
                                ptA = intersectionPts[i];
                                ptB = intersectionPts[j];
                            }
                        }
                    }

                    // find the two points that are the farthest from ptA
                    double maxDistA = 0;
                    Eigen::Vector3d ptC, ptD;
                    for (int i = 0; i < intersectionPts.size(); i++)
                    {
                        double dist = (ptA - intersectionPts[i]).norm();
                        if (dist > maxDistA)
                        {
                            maxDistA = dist;
                            ptC = intersectionPts[i];
                        }
                    }

                    // find the two points that are the farthest from ptB
                    double maxDistB = 0;
                    for (int i = 0; i < intersectionPts.size(); i++)
                    {
                        double dist = (ptB - intersectionPts[i]).norm();
                        if (dist > maxDistB)
                        {
                            maxDistB = dist;
                            ptD = intersectionPts[i];
                        }
                    }

                    // create two polygons
                    std::vector<Eigen::Vector3d> polygonPts1, polygonPts2;
                    polygonPts1.push_back(ptA);
                    polygonPts1.push_back(ptC);
                    polygonPts1.push_back(ptD);
                    polygonPts1.push_back(ptB);

                    polygonPts2.push_back(ptC);
                    polygonPts2.push_back(ptA);
                    polygonPts2.push_back(ptB);
                    polygonPts2.push_back(ptD);

                    TSPolygon tempPoly1 = TSPolygon(polygonPts1, this->m_Timber->getTSRTagsStripes()[i]->getMeanPlane());
                    tempPoly1.reorderClockwisePoints();
                    polygons.push_back(tempPoly1);

                    TSPolygon tempPoly2 = TSPolygon(polygonPts2, this->m_Timber->getTSRTagsStripes()[i]->getMeanPlane());
                    tempPoly2.reorderClockwisePoints();
                    polygons.push_back(tempPoly2);


                }
                else
                {
                    TSPolygon tempPoly = TSPolygon(intersectionPts, this->m_Timber->getTSRTagsStripes()[i]->getMeanPlane());
                    tempPoly.reorderClockwisePoints();
                    polygons.push_back(tempPoly);
                }

                intersectionPts.clear();
            }
        }

        //////////////////// ATTEMPT 1 ////////////////////

    //     for (int i = 0; i < segmentsGrouped.size(); i++)
    //     {
    //         for (auto& segA : segmentsGrouped[i])
    //         {
    //             for (auto& segB : allSegments)
    //             {
    //                 if (segA == segB) continue;

    //                 bool isIntersect = false;
    //                 Eigen::Vector3d intersectionPt;
    //                 isIntersect = segA.intersect(segB, intersectionPt);

    //                 if (isIntersect)
    //                 {
    //                     // check the intersection point is unique
    //                     bool isUnique = true;
    //                     for (auto& pt : intersectionPts)
    //                     {
    //                         if (pt.isApprox(intersectionPt, 1e-6))
    //                         {
    //                             isUnique = false;
    //                             break;
    //                         }
    //                     }
    //                     if (isUnique) intersectionPts.push_back(intersectionPt);
    //                 }

    //             }
    //         }

    //         // create a polygon from the intersection points
    //         if (intersectionPts.size() > 0)
    //         {
    //             // if (i==4)  // DEBUG
    //             // {
    //             TSPolygon tempPoly = TSPolygon(intersectionPts, this->m_PlnAABBPolygons[i].getLinkedPlane());
    //             tempPoly.reorderClockwisePoints();
    //             polygons.push_back(tempPoly);
    //             intersectionPts.clear();
    //             // }
                
    //             intersectionPts.clear();  // DEBUG

    //         }
    //     }
    }
    void TSGeometricSolver::rSelectFacePolygons(std::vector<TSPolygon>& polygons,
                                                std::vector<TSPolygon>& facePolygons,
                                                double tolerance)
    {
        for (auto& poly : polygons)
        {
            for (auto& tag : this->m_Timber->getPlaneTags())
            {
                Eigen::Vector3d& tagCtr = tag.getCenter();
                TSPlane& polyPln = poly.getLinkedPlane();

                if (polyPln.distance(tagCtr) < tolerance)
                {
                    Eigen::Vector3d tagCtrProj = polyPln.projectPoint(tagCtr);

                    if (poly.isPointInsidePolygon(tagCtrProj))
                    {
                        facePolygons.push_back(poly);
                        break;
                    }
                }
            }
        }
    }

    void TSGeometricSolver::rCreateMesh()
    {
        this->rJoinPolygons(this->m_FacePolygons, this->m_MeshOut);
        if (!this->rCheckMeshSanity(this->m_MeshOut))
            throw std::runtime_error("[ERROR]: mesh is not valid.");
    }
    void TSGeometricSolver::rJoinPolygons(std::vector<TSPolygon>& facePolygons,
                                          open3d::geometry::TriangleMesh& mesh)
    {
        for (auto& poly : this->m_FacePolygons)
        {
            mesh += poly.cvtPoly2O3dMesh();
            mesh.MergeCloseVertices(0.001);
            mesh.RemoveDuplicatedTriangles();
            mesh.RemoveDuplicatedVertices();
            mesh.RemoveNonManifoldEdges();
            mesh.RemoveDegenerateTriangles();
        }
    }
    bool TSGeometricSolver::rCheckMeshSanity(open3d::geometry::TriangleMesh& mesh)
    {
        if (mesh.HasVertices() && mesh.HasTriangles() && mesh.IsWatertight())
            return true;
        else
            return false;
    }

    void TSGeometricSolver::exportMesh2PLY(std::string& dir, std::string& name)
    {
        std::string plyName = dir + name;
        open3d::io::WriteTriangleMesh(plyName, this->m_MeshOut);
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