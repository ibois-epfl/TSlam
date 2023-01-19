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


        this->rIntersectSplittingSegments(this->m_SplitSegmentsGrouped, this->m_SplitPolygons);  // ORI

        std::cout << "size of split polygons: " << this->m_SplitPolygons.size() << std::endl;  // DEBUG

        // // get all segments in a vector
        // std::vector<TSSegment> allSegments;
        // for (auto& segs : this->m_SplitSegmentsGrouped)
        //     allSegments.insert(allSegments.end(), segs.begin(), segs.end());
        // this->rSplitPolygons(this->m_PlnAABBPolygons, this->m_SplitPolygons, allSegments);

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
        
        // uint __EXTERN = 0;
        // uint __INTERN = 1;
        // uint __NONE = 2;
        // a vector of maps
        // //      idx seg            pt's segs                pt coords
        // std::map<uint, std::tuple<std::array<uint, 2>>, Eigen::Vector3d> ptsMap = {};

        //  seg pt          pt's segs   pt coords
        std::vector<std::tuple<std::array<uint, 2>, Eigen::Vector3d>> ptsScan;

        std::vector<Eigen::Vector3d> pts_temp;
        std::vector<std::array<uint, 2>> segsIdx_temp;


        
        for (uint i = 0; i < segmentsGrouped.size(); i++)
        {
            
            auto& segsGroup = segmentsGrouped[i];

            ptsScan.clear();  // TEST

            // (a) segs x segs
            for(uint k = 0; k < segsGroup.size(); k++)
            {
                pts_temp.clear();  // TEST
                segsIdx_temp.clear();  // TEST

                // (a.1) intersect one segment with all the others
                for (uint j = 0; j < segsGroup.size(); j++)
                {
                    if (i == j) continue;

                    bool isIntersect = false;
                    Eigen::Vector3d interPt;
                    isIntersect = segsGroup[k].intersect(segsGroup[j], interPt);

                    if (isIntersect)
                    {
                        bool isUnique = true;
                        for (auto& pt : pts_temp)
                        {
                            if (pt.isApprox(interPt, this->m_EPS))
                            {
                                isUnique = false;
                                break;
                            }
                        }
                        if (isUnique)
                        {
                            pts_temp.push_back(interPt);  // TEST
                            std::array<uint, 2> segsIdx = {k, j};
                            segsIdx_temp.push_back(segsIdx);  // TEST
                        }
                    }
                }

                // (b) order the intersection points and the segsIdx by distance from the seg's origin
                // and apply the same sorting to the segsIdx arrays
                if (pts_temp.size() == 0)
                    continue;

                // std::cout << "BEFORE SORTING" << std::endl;  // DEBUG
                // std::cout << "origin coords: " << segsGroup[k].P1.transpose() << std::endl;  // DEBUG
                // for (auto& pt : pts_temp)
                //     std::cout << "pt: " << pt.transpose() << std::endl;  // DEBUG
                
                auto& segOrigin = segsGroup[k].P1;
                std::sort(pts_temp.begin(), pts_temp.end(),
                        [&segOrigin](const Eigen::Vector3d& a, const Eigen::Vector3d& b)
                        {
                            return (a - segOrigin).norm() < (b - segOrigin).norm();
                        });
                
                std::vector<std::array<uint, 2>> segsIdx_temp_sorted;
                for (auto& pt : pts_temp)
                {
                    for (uint i = 0; i < segsIdx_temp.size(); i++)
                    {
                        if (pt.isApprox(pts_temp[i], this->m_EPS))
                        {
                            segsIdx_temp_sorted.push_back(segsIdx_temp[i]);
                            break;
                        }
                    }
                }
                assert(segsIdx_temp_sorted.size() == pts_temp.size());
                segsIdx_temp = segsIdx_temp_sorted;

                std::cout << "--- size of pts_temp: " << pts_temp.size() << std::endl;  // DEBUG


                // std::cout << "AFTER SORTING" << std::endl;  // DEBUG
                // for (auto& pt : pts_temp)
                //     std::cout << "pt: " << pt.transpose() << std::endl;  // DEBUG


                

                // (c) store the sorted vectors in the ptsScan vector
                for (uint y = 0; y < pts_temp.size(); y++)
                    ptsScan.push_back(std::make_tuple(segsIdx_temp[y], pts_temp[y]));


                

                std::cout << "next seg id: " << k << "-------------------" << std::endl;  // DEBUG
            }

            // (*) remove duplicate points in ptsScan (cleaning) - TODO: find the cause
            std::vector<std::tuple<std::array<uint, 2>, Eigen::Vector3d>> ptsScan_temp;
            for (auto& pt : ptsScan)
            {
                bool isUnique = true;
                for (auto& pt2 : ptsScan_temp)
                {
                    if (std::get<1>(pt).isApprox(std::get<1>(pt2), this->m_EPS))
                    {
                        isUnique = false;
                        break;
                    }
                }
                if (isUnique)
                    ptsScan_temp.push_back(pt);
            }
            ptsScan = ptsScan_temp;

            ///////////////////////////////////////////////////////////////////////
            // where the magic happens

            std::cout << "number of inter points in map: " << ptsScan.size() << std::endl;  // DEBUG
            
            if (ptsScan.size() <= 2)
                continue;
            else if(ptsScan.size() <= 4)
            {
                // TEST: get all the points from ptsScan
                std::vector<Eigen::Vector3d> pts_temp2;
                for (auto& pt : ptsScan)
                {
                    pts_temp2.push_back(std::get<1>(pt));
                }

                TSPolygon tempPoly = TSPolygon(pts_temp2, this->m_PlnAABBPolygons[i].getLinkedPlane());
                tempPoly.reorderClockwisePoints();
                polygons.push_back(tempPoly);
            }
            else if (ptsScan.size() >= 6)
            {
                // print all the values in ptsScan
                for (auto& pt : ptsScan)
                {
                    // //  seg pt          pt's segs   pt coords
                    // std::vector<std::tuple<std::array<uint, 2>, Eigen::Vector3d>> ptsScan;

                    std::cout << "pt: " << std::get<1>(pt).transpose() << std::endl;  // DEBUG
                    std::cout << "segsIdx: " << std::get<0>(pt)[0] << ", " << std::get<0>(pt)[1] << std::endl;  // DEBUG
                }

                int i = 0;
                std::vector<uint> visitedPtsIdx;

                do
                {
                    // register current index
                    if (std::find(visitedPtsIdx.begin(), visitedPtsIdx.end(), i) != visitedPtsIdx.end())
                    {
                        i++;
                        continue;
                    }
                    visitedPtsIdx.push_back(i);

                    // (1) first pt: pick idx
                    Eigen::Vector3d& pt0 = std::get<1>(ptsScan[i]);

                    // (2) second pt: get the closest point on the same segment
                    visitedPtsIdx.push_back(i+1);
                    Eigen::Vector3d pt1 = std::get<1>(ptsScan[i+1]);

                    uint nextPtSegmentIdx = std::get<0>(ptsScan[i])[1];

                    // (2) get the second point on a different segment
                    // Eigen::Vector3d pt1 = std::get<1>(ptsScan[nextPtIdx]);


                    // update counter
                    i++;
                } while (ptsScan.size() == visitedPtsIdx.size());

                // (a) get the first point
                Eigen::Vector3d& pt0 = std::get<1>(ptsScan[0]);
                std::array<uint, 2>& segsIdx0 = std::get<0>(ptsScan[0]);

                // (b) get the second point


            }
        }
    }
    // TODO: test, dev
    void TSGeometricSolver::rSplitPolygons(std::vector<TSPolygon>& polygons,
                                           std::vector<TSPolygon>& splitPolygons,
                                           std::vector<TSSegment>& segments)
    {
        for (auto& poly : polygons)
        {
            splitPolygons.push_back(poly);
            // std::vector<TSPolygon> splitPolygons = {poly};
            std::vector<TSPolygon> tempSplitPolygons;
            std::tuple<TSPolygon, TSPolygon> subTuplePolys;

            bool isSubSplitVT;
            bool isUnique = true;
            uint NSubSplit = 0;

            do
            {
                for (int i = 0; i < splitPolygons.size(); i++)
                {
                    NSubSplit = 0;

                    for (auto& seg : segments)
                    {
                        isSubSplitVT = splitPolygons[i].splitPolygon(seg, subTuplePolys);

                        if (isSubSplitVT)
                        {
                            NSubSplit++;

                            TSPolygon polySplitA = std::get<0>(subTuplePolys);
                            TSPolygon polySplitB = std::get<1>(subTuplePolys);

                            isUnique = true;
                            for (auto& poly : tempSplitPolygons)
                            {
                                if (poly == polySplitA)
                                {
                                    isUnique = false;
                                    break;
                                }
                            }
                            if (isUnique)
                                tempSplitPolygons.push_back(polySplitA);

                            isUnique = true;
                            for (auto& poly : tempSplitPolygons)
                            {
                                if (poly == polySplitB)
                                {
                                    isUnique = false;
                                    break;
                                }
                            }
                            if (isUnique)
                                tempSplitPolygons.push_back(polySplitB);
                        }
                    }

                    if (NSubSplit == 0)
                    {
                        isUnique = true;
                        for (auto& poly : tempSplitPolygons)
                        {
                            if (poly == splitPolygons[i])
                            {
                                isUnique = false;
                                break;
                            }
                        }
                        if (isUnique)
                            tempSplitPolygons.push_back(splitPolygons[i]);
                    }
                }

                splitPolygons.clear();
                if (tempSplitPolygons.size() > 0)
                    for (auto& tpoly : tempSplitPolygons)
                        splitPolygons.emplace_back(tpoly);
                tempSplitPolygons.clear();

            } while (NSubSplit > 0);
        }
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