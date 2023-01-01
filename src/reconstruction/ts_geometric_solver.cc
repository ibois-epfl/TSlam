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

// TODO: try to catch more sensitive creaeses
// TODO: try a timber with notch and implement merging polygons at the end (when selecting
//       the polygons face candidates)

namespace tslam
{
    void TSGeometricSolver::reconstruct()
    {
        this->rDetectCreasesTags();

        this->rIntersectTagPlnAABB();
        this->rCreatePolysurface();
        // this->rCreateMesh();  // TODO: to reactivate

#ifdef TSLAM_REC_DEBUG
    // Debug visualizer
    open3d::visualization::Visualizer* vis(new open3d::visualization::Visualizer());
    vis->CreateVisualizerWindow("TSPlaneTags", 1920, 1080);

    


    // create a vector with 10 hardcoded colors
    std::vector<Eigen::Vector3d> clrs;
    for (int i = 0; i < 10; i++)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0, 1);
        Eigen::Vector3d color = Eigen::Vector3d(dis(gen), dis(gen), dis(gen));
        clrs.push_back(color);
    }

    // draw base plane tags as wireframe
    for (auto& tag : this->m_Timber->getPlaneTags())
    {
        open3d::geometry::TriangleMesh tagBase = tag.getOpen3dMesh();
        auto planeTagsLineset1 = open3d::geometry::LineSet::CreateFromTriangleMesh(tagBase);

        // if (tag.isEdge())
        //     planeTagsLineset1->PaintUniformColor(Eigen::Vector3d(0, 1, 0));
        // else
        //     planeTagsLineset1->PaintUniformColor(Eigen::Vector3d(1, 0, 0));

        planeTagsLineset1->PaintUniformColor(clrs[tag.getFaceIdx()]);

        vis->AddGeometry(planeTagsLineset1);
    }

    // // // get the tag centers as point cloud and color them according to their faceIdx value
    // std::shared_ptr<open3d::geometry::PointCloud> tagCenters = std::make_shared<open3d::geometry::PointCloud>();
    // for (auto& tag : this->m_Timber->getPlaneTags())
    // {
    //     if (tag.isEdge())
    //     {
    //         tagCenters->points_.push_back(tag.getCenter());
    //         tagCenters->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //     }
    // }
    // vis->AddGeometry(tagCenters);

    // // show normals of the tag centers
    // std::shared_ptr<open3d::geometry::LineSet> tagNormals = std::make_shared<open3d::geometry::LineSet>();
    // for (auto& tag : this->m_Timber->getPlaneTags())
    // {
    //     tagNormals->points_.push_back(tag.getCenter());
    //     tagNormals->points_.push_back(tag.getCenter() + tag.getNormal() * 1.0);
    //     tagNormals->lines_.push_back(Eigen::Vector2i(tagNormals->points_.size() - 2, tagNormals->points_.size() - 1));
    // }
    // vis->AddGeometry(tagNormals);

    // // draw AABB
    // std::shared_ptr<open3d::geometry::LineSet> aabbLineset = std::make_shared<open3d::geometry::LineSet>();

    // // draw polygon segments3D
    // for (auto& pg : this->m_PlnAABBPolygons)
    // {
    //     std::vector<Eigen::Vector3d> pts = pg.getVertices();
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

    // // draw first intersection polygon centers
    // for (auto& fpoly : this->m_PlnAABBPolygons)
    // {
    //     std::shared_ptr<open3d::geometry::PointCloud> pntCldPCtr = std::make_shared<open3d::geometry::PointCloud>();
    //     pntCldPCtr->points_.push_back(fpoly.getCenter());
    //     pntCldPCtr->PaintUniformColor(Eigen::Vector3d(0, 1, 1));
    //     vis->AddGeometry(pntCldPCtr);
    // }

    // // draw new merged planes centers
    // std::shared_ptr<open3d::geometry::PointCloud> pntCldNewPlanes = std::make_shared<open3d::geometry::PointCloud>();
    // for (auto& pg : this->m_MergedPolygons)
    // {
    //     pntCldNewPlanes->points_.push_back(pg.getCenter());
    //     pntCldNewPlanes->PaintUniformColor(Eigen::Vector3d(0.7, 0.3, 0.9));
    //     vis->AddGeometry(pntCldNewPlanes);
    // }

    // // draw new interesected polygons
    // for (auto& pg : this->m_MergedPolygons)
    // {
    //     std::vector<Eigen::Vector3d> pts = pg.getVertices();
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

    // draw all split poly
    for (auto& poly : this->m_SplitPolygons)
    {
        std::vector<Eigen::Vector3d> pts = poly.getVertices();
        // random color
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0, 1);
        Eigen::Vector3d color = Eigen::Vector3d(dis(gen), dis(gen), dis(gen));
        for (int i = 0; i < pts.size(); i++)
        {
            std::shared_ptr<open3d::geometry::Segment3D> segm = std::make_shared<open3d::geometry::Segment3D>(pts[i], pts[(i+1)%pts.size()]);
            std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
            segLineset->points_.push_back(segm->Origin());
            segLineset->points_.push_back(segm->EndPoint());
            segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
            segLineset->PaintUniformColor(color);
            vis->AddGeometry(segLineset);
        }
    }

    // // show final mesh
    // std::shared_ptr<open3d::geometry::TriangleMesh> mesh = std::make_shared<open3d::geometry::TriangleMesh>(this->m_MeshOut);
    // mesh->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
    // vis->AddGeometry(mesh);

    // std::shared_ptr<open3d::geometry::PointCloud> pcdMeshCenters = std::make_shared<open3d::geometry::PointCloud>();
    // for (auto& tri : mesh->triangles_)
    // {
    //     Eigen::Vector3d ctr = (mesh->vertices_[tri(0)] + mesh->vertices_[tri(1)] + mesh->vertices_[tri(2)]) / 3.;
    //     pcdMeshCenters->points_.push_back(ctr);
    // }
    // pcdMeshCenters->PaintUniformColor(Eigen::Vector3d(1., 0., 0.));
    // vis->AddGeometry(pcdMeshCenters);




    vis->Run();
    vis->Close();
    vis->DestroyVisualizerWindow();
#endif
    }
    // TODO: clean out this function
    void TSGeometricSolver::rDetectCreasesTags()
    {
        // DIVIDE TAGS BY STRIPE <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        auto ctrsCopy = this->m_Timber->getTagsCtrs();
        auto tagsCopy = this->m_Timber->getPlaneTags();

        std::vector<std::vector<TSRTag>> stripes = {};
        std::vector<TSRTag> stripe = {};

        open3d::geometry::KDTreeFlann kdtree;
        kdtree.SetGeometry(ctrsCopy);
        std::vector<int> indices;
        std::vector<double> distances;
        uint knn = 2;
        uint faceIdx = 0;
        uint idx = 0;
        uint nextIdx = 0;

        do
        {
            kdtree.SetGeometry(ctrsCopy);
            kdtree.SearchKNN(ctrsCopy.points_[idx], knn, indices, distances);

            nextIdx = indices[1];

            double angle = tslam::TSVector::angleBetweenVectors(
                tagsCopy[idx].getNormal(),
                tagsCopy[nextIdx].getNormal());

            if (angle < this->m_CreaseAngleThreshold)
            {
                tagsCopy[idx].setFaceIdx(faceIdx);

                stripe.push_back(tagsCopy[idx]);
            }
            else if (angle > this->m_CreaseAngleThreshold)
            {
                tagsCopy[idx].setFaceIdx(faceIdx);
                faceIdx++;
                tagsCopy[nextIdx].setFaceIdx(faceIdx);

                stripe.push_back(tagsCopy[idx]);
                stripes.push_back(stripe);
                stripe.clear();
                stripe.push_back(tagsCopy[nextIdx]);
            }

            tagsCopy.erase(tagsCopy.begin() + idx);
            ctrsCopy.points_.erase(ctrsCopy.points_.begin() + idx);
            idx = (nextIdx > idx) ? nextIdx - 1 : nextIdx;

        } while(ctrsCopy.points_.size() > 0);

        // add the last stripe
        stripes.push_back(stripe);  //TODO: test to add again

        // MArk THE EXTREMES OF THE STRIPE <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        // TODO: check out if this method of finding the extremes is correct
        for (auto& stripe : stripes)
        {
            // get the extremitiy tags
            TSRTag& minTag = stripe.front();
            TSRTag& maxTag = stripe.back();
            for (auto& tag : stripe)
            {
                tag.setType(TSRTagType::Side);
                if (tag.getCenter()(0) < minTag.getCenter()(0))
                    minTag = tag;
                if (tag.getCenter()(0) > maxTag.getCenter()(0))
                    maxTag = tag;
            }
            minTag.setType(TSRTagType::Edge);
            maxTag.setType(TSRTagType::Edge);
        }

        // TODO: test, merge stripes that belong to the same face
        // merge stripes that belong to the same face
        // to know if two stripes belong to the same face:
        // how to check if two stripes belong to the same face?
        // 1. check if the stripes are adjacent
        // 2. check if the stripes have the same normal

        std::cout << "BEFORE: Number of stripes: " << stripes.size() << std::endl;  // DEBUG

        // TODO: test it here

        std::cout << "AFTER: Number of stripes: " << stripes.size() << std::endl;  // DEBUG


        this->m_Timber->setTSRTagsStripes(stripes);  // TODO: temp, implement properly


        // store the tags <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        std::vector<TSRTag> temp = {};
        for (auto& stripe : stripes)
            for (auto& tag : stripe)
                temp.push_back(tag);
        this->m_Timber->setPlaneTags(temp);  // TODO: temp, implement properly

    }

    void TSGeometricSolver::rIntersectTagPlnAABB()
    {
        this->m_Timber->scaleAABB(this->m_AABBScaleFactor);

        this->timeStart("[PROFILER]: intersect planes with AABB");  // TODO: implement true Profiler

        Eigen::Vector3d* outPtsPtr = new Eigen::Vector3d[3*6];
        std::vector<Eigen::Vector3d>* planeIntersections = new std::vector<Eigen::Vector3d>();



        //======tempTest=========================================================================

        // here we get all the planes of each stripe of tags
        std::vector<TSPlane> planes = {};
        for (auto& stripe : this->m_Timber->getTSRTagsStripes())
        {
            // get the extremitiy tags
            // TSRTag& minTag = stripe.front();
            TSRTag& minTag = stripe.front();

            // TSRTag& maxTag = stripe.back();
            TSRTag& maxTag = stripe.back();

            // compute the normal as the mean of the normals of the tags between the extremes
            Eigen::Vector3d meanNormal = Eigen::Vector3d::Zero();
            for (auto& tag : stripe)
                meanNormal += tag.getNormal();
            meanNormal /= stripe.size();

            // fit plane through 2 points and a given normal
            TSPlane meanPlane = TSPlane(meanNormal,
                                        minTag.getCenter(),
                                        maxTag.getCenter());
            
            planes.push_back(meanPlane);
        }

        // we need to merge to much similar planes (too close to each other)
        // average the planes if they are too close
        // std::cout << "BEFORE: planes.size(): " << planes.size() << "\n";  // DEBUG
        std::vector<TSPlane> tempPlns2Merge = {};

        auto series = this->m_Timber->getTSRTagsStripes();
        std::vector<TSRTag> sTemp = {};
        

        for (int i = 0; i < planes.size(); i++)
        {
            for (int j = i + 1; j < planes.size(); j++)
            {
                // std::cout << "distance: " <<  planes[i].distance(planes[j]) << "\n";  // DEBUG
                if (planes[i].distance(planes[j]) < 5.2)
                {
                    // check if the normal of the two planes are similar
                    if (planes[i].Normal.dot(planes[j].Normal) > 0.9)
                    {
                        // // ======================== test0 ========================
                        // std::cout << "BEFORE: tempPlns2Merge size: " <<  tempPlns2Merge.size() << "\n";  // DEBUG
                        // push the planes to the temp vector
                        // tempPlns2Merge.push_back(planes[i]);
                        // tempPlns2Merge.push_back(planes[j]);
                        
                        // FIXME: we need to merge the two stripes!

                        // merge the two stripes and find the two more distant tags
                        sTemp = series[i];

                        std::cout << "BEFORE: size sTemp: " << sTemp.size() << "\n";  // DEBUG

                        sTemp.insert(sTemp.end(), series[j].begin(), series[j].end());

                        std::cout << "AFTER: size sTemp: " << sTemp.size() << "\n";  // DEBUG


                        TSRTStripe sTemp2 = TSRTStripe(sTemp);
                        std::cout << "SERIESOBJ: size sTemp: " << sTemp2.size() << "\n";  // DEBUG


                        // mean normal FIXME: this has to be stripe class method
                        Eigen::Vector3d meanNormal = Eigen::Vector3d::Zero();
                        for (auto& tag : sTemp)
                            meanNormal += tag.getNormal();
                        meanNormal /= sTemp.size();

                        // get the extremitiy tags
                        sTemp2.reorderTags();

                        TSRTag minTag = sTemp2.front();
                        TSRTag maxTag = sTemp2.back();


                        TSPlane avgPlane = TSPlane(meanNormal, 
                                                   minTag.getCenter(), 
                                                   maxTag.getCenter());

                        // tempPlns2Merge.push_back(avgPlane);  // TEST/DEBUG
                        // goto testend;  // TEST/DEBUG


                        planes[i] = avgPlane;
                        series.erase(series.begin() + j);  // TODO: test
                        planes.erase(planes.begin() + j);
                        j--;
                        sTemp.clear();  // TODO: test


                        // std::cout << "AFETR: tempPlns2Merge size: " <<  tempPlns2Merge.size() << "\n";  // DEBUG
                    }
                }
            }
        }
        // testend:                  // TEST/DEBUG
        // planes.clear();           // TEST/DEBUG
        // planes = tempPlns2Merge;  // TEST/DEBUG
        // std::cout << "AFTER: planes.size(): " << planes.size() << "\n";  // DEBUG


        for (int i = 0; i < planes.size(); i++)
        {
            // TSRTag& minTag = stripe.front();
            // TSRTag& maxTag = stripe.back();

            // mean the min and max tags' planes
            TSPlane meanPlane = planes[i];

            unsigned int outPtsCount;
            tslam::TSPlane::plane2AABBSegmentIntersect(meanPlane,
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

            TSPolygon tempPoly = TSPolygon(*planeIntersections, meanPlane);
            tempPoly.reorderClockwisePoints();
            this->m_PlnAABBPolygons.push_back(tempPoly);
        }
        delete outPtsPtr;
        delete planeIntersections;




        //======working=========================================================================


        // for (auto& stripe : this->m_Timber->getTSRTagsStripes())
        // {
        //     TSRTag& minTag = stripe.front();
        //     TSRTag& maxTag = stripe.back();

        //     // mean the min and max tags' planes
        //     TSPlane meanPlane = TSPlane(minTag.getNormal(), minTag.getCenter(), maxTag.getCenter());

        //     unsigned int outPtsCount;
        //     tslam::TSPlane::plane2AABBSegmentIntersect(meanPlane,
        //                                                this->m_Timber->getAABB().min_bound_,
        //                                                this->m_Timber->getAABB().max_bound_,
        //                                                outPtsPtr,
        //                                                outPtsCount);
        //     // b. save the result into a polygon
        //     planeIntersections->reserve(outPtsCount);
        //     planeIntersections->clear();
        //     for (unsigned int i = 0; i < outPtsCount; i++)
        //     {
        //         planeIntersections->push_back(outPtsPtr[i]);
        //     }

        //     TSPolygon tempPoly = TSPolygon(*planeIntersections, meanPlane);
        //     tempPoly.reorderClockwisePoints();
        //     this->m_PlnAABBPolygons.push_back(tempPoly);
        // }
        // delete outPtsPtr;
        // delete planeIntersections;


        //======oRIGINAL=========================================================================
        // for (auto& t : this->m_Timber->getPlaneTags())
        // {
        //     // a. skip face tags
        //     if (t.isSide()) continue;

        //     // b. caculate the intersection points
        //     unsigned int outPtsCount;
        //     tslam::TSPlane::plane2AABBSegmentIntersect(t.getPlane(),
        //                                                this->m_Timber->getAABB().min_bound_,
        //                                                this->m_Timber->getAABB().max_bound_,
        //                                                outPtsPtr,
        //                                                outPtsCount);
        //     // b. save the result into a polygon
        //     planeIntersections->reserve(outPtsCount);
        //     planeIntersections->clear();
        //     for (unsigned int i = 0; i < outPtsCount; i++)
        //     {
        //         planeIntersections->push_back(outPtsPtr[i]);
        //     }

        //     TSPolygon tempPoly = TSPolygon(*planeIntersections, t.getPlane());
        //     tempPoly.reorderClockwisePoints();
        //     this->m_PlnAABBPolygons.push_back(tempPoly);
        // }
        // delete outPtsPtr;
        // delete planeIntersections;
        //========================================================================================

        // c. mean the polygons into new one
        // this->rMeanPolygonPlanes();
        this->m_MergedPolygons = this->m_PlnAABBPolygons;  // TODO: test

        // d. intersect the new polygons with the timber
        this->rIntersectMeanPolygonPlnAABB();

        this->timeEnd();  // TODO: implement true Profiler
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
                    // if it has, check the next one
                    continue;
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
    void TSGeometricSolver::rIntersectMeanPolygonPlnAABB()
    {
        Eigen::Vector3d* outPtsPtr2 = new Eigen::Vector3d[3*6];
        for (auto& mpoly : this->m_MergedPolygons)
        {
            // a. caculate the intersection points
            unsigned int outPtsCount2;
            tslam::TSPlane::plane2AABBSegmentIntersect(mpoly.getLinkedPlane(),
                                                       this->m_Timber->getAABB().min_bound_,
                                                       this->m_Timber->getAABB().max_bound_,
                                                       outPtsPtr2,
                                                       outPtsCount2);

            // b. store the new itnersection points into the polygon
            for (unsigned int i = 0; i < outPtsCount2; i++)
            {
                mpoly.addVertex(outPtsPtr2[i]);
            }
            mpoly.reorderClockwisePoints();
        }
        delete outPtsPtr2;
    }


    void TSGeometricSolver::rCreatePolysurface()
    {
        this->rIntersectPolygons(this->m_MergedPolygons, this->m_SplitSegments);

        this->rSplitPolygons(this->m_MergedPolygons,
                             this->m_SplitPolygons,
                             this->m_SplitSegments);

        this->rSelectFacePolygons(this->m_SplitPolygons,
                                  this->m_FacePolygons,
                                  this->m_MaxPolyTagDist);
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
                    if (s == seg)
                    {
                        isIntersect = false;
                        break;
                    }
                }
                if (isIntersect) segments.push_back(seg);
            }
        }
    }
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
}