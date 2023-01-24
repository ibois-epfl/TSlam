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

// TODO: place in ts_geometric_solver.hh
#include "ts_tassellation.hh"

namespace tslam
{
    void TSGeometricSolver::reconstruct()
    {
        this->rDetectFacesStripes();
        this->rIntersectStripeTagPlnAABB();
        this->rCreatePolysurface();
        this->rCreateMesh();

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
                // assign a random color to each face
                Eigen::Vector3d color = Eigen::Vector3d((double)rand() / RAND_MAX, (double)rand() / RAND_MAX, (double)rand() / RAND_MAX);

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
                    segLineset->PaintUniformColor(color);
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


        this->rIntersectSplittingSegments(this->m_PlnAABBPolygons,
                                          this->m_SplitPolygons,
                                          this->m_SplitSegmentsGrouped);

        // std::cout << "size of split polygons: " << this->m_SplitPolygons.size() << std::endl;  // DEBUG
        // this->m_FacePolygons = this->m_SplitPolygons;  // DEBUG

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
                this->m_SplitSegmentsPlanes.push_back(polyA.getLinkedPlane());  // FIXME: test, if works move to func params
                segmentsGrouped.push_back(tempSegments);
                tempSegments.clear();
            }
        }

        // (*b) intersect the segments among themselves and if they have only one intersection point discard them
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
    void TSGeometricSolver::rIntersectSplittingSegments(std::vector<TSPolygon> &AabbPolygons,
                                                        std::vector<TSPolygon> &polygons,
                                                        std::vector<std::vector<TSSegment>> &segmentsGrouped)
    {
        std::cout << "<<<<<<<<<< rIntersectSplittingSegments >>>>>>>>>>" << std::endl;  // DEBUG

        // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DEBUG
        // deep copy of the segmentsGrouped
        // std::vector<std::vector<TSSegment>> segmentsGroupedCopy;
        // for (auto& segGroup : segmentsGrouped)
        // {
        //     std::vector<TSSegment> tempSegGroup;
        //     for (auto& seg : segGroup)
        //     {
        //         tempSegGroup.push_back(seg);
        //     }
        //     segmentsGroupedCopy.push_back(tempSegGroup);
        // }
        // segmentsGrouped.clear();
        // segmentsGrouped.push_back(segmentsGroupedCopy[1]);
        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< DEBUG

        // TEST FOR TRANSFOM
        TSPlane worldXYPlane = TSPlane(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 0));
        std::vector<Eigen::Matrix3d> invMats;

        std::cout << "number of segments groups: " << segmentsGrouped.size() << std::endl;  // DEBUG
        std::cout << "number of polygons: " << AabbPolygons.size() << std::endl;  // DEBUG
        std::cout << "number of planes of splitting segments: " << this->m_SplitSegmentsPlanes.size() << std::endl;  // DEBUG

        for (uint i = 0; i < segmentsGrouped.size(); i++)
        {
            // DEBUG: print number of segments per group
            std::cout << "group " << i << " has " << segmentsGrouped[i].size() << " segments" << std::endl;

            auto& segs = segmentsGrouped[i];
            TSPlane& planeA = this->m_SplitSegmentsPlanes[i];
            // planeA.Normal.normalize();
            Eigen::Matrix3d rot = TSPlane::getPlane2XYPlaneRotation(planeA);  // TEST 1
            // Eigen::Matrix3d rot = TSPlane::getRotationToPlaneXYMatrix(planeA, worldXYPlane);  // TEST 2


            // to origin translation

            // Eigen::Matrix3d transl = mat.block<3, 1>(0, 3);


            // mat.normalize();

            Eigen::Matrix3d invMat = rot.inverse();
            invMats.push_back(invMat);

            for (auto& seg : segs)
            {
                // move first to origin
                seg.transform(rot);
            }
        }

        std::vector<std::vector<TSPolygon>> polygonsGrouped;
        std::shared_ptr<TSTassellation> tassellatorPtr = std::make_shared<TSTassellation>(TSTassellation());
        tassellatorPtr->tassellate(segmentsGrouped, polygonsGrouped);

        // store the polygons tasselation
        for (uint i = 0; i < polygonsGrouped.size(); i++)
        {
            for (uint j = 0; j < polygonsGrouped[i].size(); j++)
            {
                polygonsGrouped[i][j].transform(invMats[i]);
                polygonsGrouped[i][j].setLinkedPlane(this->m_SplitSegmentsPlanes[i]);
                polygons.push_back(polygonsGrouped[i][j]);
            }
        }

        // // TEST: reverting segments to their original position
        // for (uint i = 0; i < segmentsGrouped.size(); i++)
        // {
        //     auto& segs = segmentsGrouped[i];
        //     for (auto& seg : segs)
        //     {
        //         seg.transform(invMats[i]);
        //     }
        // }

        // // TEST FOR INV TRANSFOM
        // for (uint i = 0; i < polygonsGrouped.size(); i++)
        // {
        //     for (uint j = 0; j < polygonsGrouped[i].size(); j++)
        //     {
        //         polygonsGrouped[i][j].transform(invMats[i]);
        //         polygonsGrouped[i][j].setLinkedPlane(AabbPolygons[i].getLinkedPlane());
        //         polygons.push_back(polygonsGrouped[i][j]);
        //     }
        // }
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