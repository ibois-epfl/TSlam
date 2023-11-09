/**
This file is part of TSLAM and distributed under the terms of the GPLv3.

Copyright (C) 2023 Andrea Settimi <andrea dot settimi at epfl dot ch>, Hong-Bin Yang <hong dash bin dot yang at epfl dot ch> (IBOIS, EPFL)

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Andrea Settimi and Hong-Bin Yang ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied Andrea Settimi and Hong-Bin Yang.
*/
#include "ts_geometric_solver.hh"

#ifdef TSLAM_REC_O3D_VISUAL_DEBUG
    #include<open3d/Open3D.h>
#endif
#include <stdexcept>
#include <algorithm>
#include <math.h>
#include <map>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <random>
#include <array>

// TODO: test if working clustering by normals
#include <cilantro/clustering/connected_component_extraction.hpp>
#include <cilantro/utilities/point_cloud.hpp>
#include <cilantro/utilities/timer.hpp>
#include <cilantro/visualization.hpp>

namespace tslam::Reconstruction
{
    void TSGeometricSolver::reconstruct()
    {
        this->rDetectFacesStripes();
        this->rIntersectStripeTagPlnAABB();
        this->rCreatePolysurface();
        this->rCreateMesh();

#ifdef TSLAM_REC_O3D_VISUAL_DEBUG
        this->visualize(this->m_ShowVisualizer,
                        this->m_DrawTags,
                        this->m_DrawTagNormals,
                        this->m_DrawAabb,
                        this->m_DrawIntersectedPoly,
                        this->m_DrawSplittingSegments,
                        this->m_DrawSelectedFace,
                        this->m_DrawFinalMesh);
#endif
    }

#ifdef TSLAM_REC_O3D_VISUAL_DEBUG
    void TSGeometricSolver::visualize(bool showVisualizer,
                                      bool drawTags,
                                      bool drawTagNormals,
                                      bool drawAabb,
                                      bool drawIntersectedPoly,
                                      bool drawSplittingSegments,
                                      bool drawSelectedFaces,
                                      bool drawFinalMesh)
    {
        if (!showVisualizer)
            return;
        open3d::visualization::Visualizer* vis(new open3d::visualization::Visualizer());
        vis->CreateVisualizerWindow("TSPlaneTags", 1920, 1080);

        if (drawTags)
        {
            std::vector<Eigen::Vector3d> clrs;
            for (uint i = 0; i < this->m_Timber.getTSRTagsStripes().size(); i++)
            {
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_real_distribution<> dis(0, 1);
                Eigen::Vector3d color = Eigen::Vector3d(dis(gen), dis(gen), dis(gen));
                clrs.push_back(color);
            }

            for (uint j = 0; j < this->m_Timber.getTSRTagsStripes().size(); j++)
            {
                auto& clr = clrs[j];
                auto& stripe = this->m_Timber.getTSRTagsStripes()[j];
                for (auto& tag : *stripe)
                {
                    open3d::geometry::TriangleMesh tagBase;
                    tagBase.vertices_.push_back(tag.getCornerA());
                    tagBase.vertices_.push_back(tag.getCornerB());
                    tagBase.vertices_.push_back(tag.getCornerC());
                    tagBase.triangles_.push_back(Eigen::Vector3i(0, 1, 2));
                    tagBase.vertices_.push_back(tag.getCornerC());
                    tagBase.vertices_.push_back(tag.getCornerD());
                    tagBase.vertices_.push_back(tag.getCornerA());
                    tagBase.triangles_.push_back(Eigen::Vector3i(3, 4, 5));

                    auto planeTagsLineset1 = open3d::geometry::LineSet::CreateFromTriangleMesh(tagBase);
                    planeTagsLineset1->PaintUniformColor(clr);

                    vis->AddGeometry(planeTagsLineset1);
                }
            }
        }

        if (drawTagNormals)
        {
            std::shared_ptr<open3d::geometry::LineSet> tagNormals = std::make_shared<open3d::geometry::LineSet>();
            for (auto& tag : this->m_Timber.getPlaneTags())
            {
                tagNormals->points_.push_back(tag.getCenter());
                tagNormals->points_.push_back(tag.getCenter() + tag.getNormal() * 1.0);
                tagNormals->lines_.push_back(Eigen::Vector2i(tagNormals->points_.size() - 2, tagNormals->points_.size() - 1));
            }
            vis->AddGeometry(tagNormals);
        }
        
        if (drawAabb)
        {
            std::shared_ptr<open3d::geometry::LineSet> aabbLineset = std::make_shared<open3d::geometry::LineSet>();

            Eigen::Vector3d p1 = this->m_Timber.getAABBMin();
            Eigen::Vector3d p2 = this->m_Timber.getAABBMax();

            aabbLineset->points_.push_back(p1);
            aabbLineset->points_.push_back(p2);
            aabbLineset->lines_.push_back(Eigen::Vector2i(0, 1));

            vis->AddGeometry(aabbLineset);
        }

        if (drawIntersectedPoly)
        {
            for (uint j = 0; j < this->m_PlnAABBPolygons.size(); j++)
            {
                std::vector<Eigen::Vector3d> pts = this->m_PlnAABBPolygons[j].getVertices();
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
            for (uint i = 0; i < this->m_SplitSegmentsGrouped.size(); i++)
            {
                std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
                for (auto& segm : this->m_SplitSegmentsGrouped[i])
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

                // draw normal of the face
                std::shared_ptr<open3d::geometry::LineSet> crossLineset = std::make_shared<open3d::geometry::LineSet>();
                crossLineset->points_.push_back(pg.getCenter());
                crossLineset->points_.push_back(pg.getCenter() + pg.getNormal() * 1.0);
                crossLineset->lines_.push_back(Eigen::Vector2i(0, 1));
                crossLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
                crossLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
                crossLineset->PaintUniformColor(color);
                vis->AddGeometry(crossLineset);
            }
        }
        
        if (drawFinalMesh)
        {
            std::shared_ptr<open3d::geometry::TriangleMesh> mesh = std::make_shared<open3d::geometry::TriangleMesh>(this->m_MeshOutO3d);
            mesh->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
            vis->AddGeometry(mesh);
        }

        vis->Run();
        vis->Close();
        vis->DestroyVisualizerWindow();
    }
#endif

    void TSGeometricSolver::rDetectFacesStripes()
    {
        std::vector<std::shared_ptr<TSRTStripe>> tempStripes;

        this->clusterStripesByNormal(tempStripes,
                                     this->m_RadiusSearch,
                                     this->m_CreaseAngleThreshold,
                                     this->m_MinClusterSize
        );

        this->rRefineStripesByPlanes(tempStripes);

        this->m_Timber.setTSRTagsStripes(tempStripes);
    }
    void TSGeometricSolver::clusterStripesByNormal(std::vector<std::shared_ptr<TSRTStripe>>& stripesGrouped,
                                                   float radius, float threshDeg, int minClusterSize)
    {
        uint nbrTags = this->m_Timber.getPlaneTags().size();
        const std::vector<TSRTag>& tags = this->m_Timber.getPlaneTags();

        cilantro::VectorSet3f ctrs(3, nbrTags);
        for (int i = 0; i < nbrTags; i++)
            ctrs.col(i) = this->m_Timber.getPlaneTags()[i].getCenter().cast<float>();
        cilantro::VectorSet3f normals(3, nbrTags);
        for (int i = 0; i < nbrTags; i++)
            normals.col(i) = this->m_Timber.getPlaneTags()[i].getNormal().cast<float>();
        cilantro::VectorSet3f colors(3, nbrTags);
        for (int i = 0; i < nbrTags; i++)
            colors.col(i) = Eigen::Vector3f(0.5, 0.5, 0.5);

        // convert pts vector to cilantro point cloud
        cilantro::PointCloud3f cloud(ctrs, normals, colors);

        // run clustering
        cilantro::RadiusNeighborhoodSpecification<float> nh(radius * radius);
        cilantro::NormalsProximityEvaluator<float, 3> ev(cloud.normals, (float)(threshDeg * M_PI / 180.0));

        cilantro::ConnectedComponentExtraction3f<> cce(cloud.points);
        cce.segment(nh, ev, minClusterSize, cloud.size());

        // create a vector of stripes based on the number of clusters
        size_t nbrLabels = cce.getNumberOfClusters();
        const auto& labels = cce.getPointToClusterIndexMap();

        // create a vector of stripes based on the number of clusters
        std::vector<std::shared_ptr<TSRTStripe>> stripes = {};

        for (uint idx = 0; idx < nbrLabels; idx++)
        {
            std::vector<TSRTag> stripeTags = std::vector<TSRTag>();
            for (uint j = 0; j < nbrTags; j++)
            {
                if (labels[j] == idx)
                    stripeTags.push_back(tags[j]);
            }
            std::shared_ptr<TSRTStripe> stripePtr = std::make_shared<TSRTStripe>(stripeTags);
            stripes.push_back(stripePtr);
        }

        // reorder the tags on each stripe
        for (auto& stripe : stripes)
            stripe->reorderTags();

        // assign the stripes to the output
        stripesGrouped = stripes;
    }
    void TSGeometricSolver::rRefineStripesByPlanes(std::vector<std::shared_ptr<TSRTStripe>>& stripesGrouped)
    {
        for (int i = 0; i < stripesGrouped.size(); i++)
        {
            for (int j = i + 1; j < stripesGrouped.size(); j++)
            {
                double dist = stripesGrouped[i]->getMeanPlane().distance(stripesGrouped[j]->getMeanPlane());
                double angle = stripesGrouped[i]->getMeanPlane().Normal.dot(stripesGrouped[j]->getMeanPlane().Normal);
                double angleDeg = acos(angle) * 180. / M_PI;

                if (stripesGrouped[i]->getMeanPlane().distance(stripesGrouped[j]->getMeanPlane()) < this->m_MaxPlnDist2Merge &&
                    (angleDeg < this->m_MaxPlnAngle2Merge || angleDeg > 180 - this->m_MaxPlnAngle2Merge)
                    )
                {
                    *stripesGrouped[i] += *stripesGrouped[j];

                    TSPlane avgPlane = TSPlane(stripesGrouped[i]->getNormal(), 
                                                stripesGrouped[i]->front().getCenter(), 
                                                stripesGrouped[i]->back().getCenter());

                    stripesGrouped[i]->setMeanPlane(avgPlane);

                    stripesGrouped.erase(stripesGrouped.begin() + j);
                    j--;
                }
            }
        }
    }

    void TSGeometricSolver::rIntersectStripeTagPlnAABB()
    {
        // scale the AABB
        this->m_Timber.scaleAABB(this->m_AABBScaleFactor);

        // intersect the planes with the timber's AABB
        Eigen::Vector3d* outPtsPtr = new Eigen::Vector3d[3*6];
        std::vector<Eigen::Vector3d>* planeIntersections = new std::vector<Eigen::Vector3d>();

        for (int i = 0; i < this->m_Timber.getTSRTagsStripes().size(); i++)
        {
            // a. intersect the plane with the AABB
            unsigned int outPtsCount;
            const TSPlane& meanStripePlane = this->m_Timber.getTSRTagsStripes()[i]->getMeanPlane();
            
            TSPlane::plane2AABBSegmentIntersect(meanStripePlane,
                                                this->m_Timber.getAABBMin(),
                                                this->m_Timber.getAABBMax(),
                                                outPtsPtr,
                                                outPtsCount);
            
            // b. save the result into a polygon
            planeIntersections->reserve(outPtsCount);
            planeIntersections->clear();
            for (unsigned int i = 0; i < outPtsCount; i++)
                planeIntersections->push_back(outPtsPtr[i]);

            TSPolygon tempPoly = TSPolygon(*planeIntersections, meanStripePlane);
            tempPoly.reorderClockwisePoints();
            this->m_PlnAABBPolygons.push_back(tempPoly);
        }
        delete outPtsPtr;
        delete planeIntersections;

        // (*)delete duplicates from the polygons list (e.g. double stripes on the same plane for synthetic data)
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

        // (**) delete polygons with less than 3 verticees
        for (int i = 0; i < this->m_PlnAABBPolygons.size(); i++)
        {
            if (this->m_PlnAABBPolygons[i].getVertices().size() < 3)
            {
                this->m_PlnAABBPolygons.erase(this->m_PlnAABBPolygons.begin() + i);
                i--;
            }
        }
    }

    void TSGeometricSolver::rCreatePolysurface()
    {
        this->rIntersectPolygons(this->m_PlnAABBPolygons,
                                 this->m_SplitSegmentsGrouped,
                                 this->m_SplitSegmentsPlanes);

        this->rTassellateSplittingSegments(this->m_SplitPolygons,
                                           this->m_SplitSegmentsPlanes,
                                           this->m_SplitSegmentsGrouped);

        this->rSelectFacePolygons(this->m_SplitPolygons,
                                  this->m_FacePolygons,
                                  this->m_MaxPolyTagDist,
                                  this->m_MaxPlnAngle2Merge
                                  );
    }
    void TSGeometricSolver::rIntersectPolygons(std::vector<TSPolygon> &polygons,
                                               std::vector<std::vector<TSSegment>> &segmentsGrouped,
                                               std::vector<TSPlane> &planes)
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
                planes.push_back(polyA.getLinkedPlane());
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

        // (c) remove duplicate segments
        for (auto& segGroup : segmentsGrouped)
        {
            for (uint i = 0; i < segGroup.size(); i++)
            {
                for (uint j = i + 1; j < segGroup.size(); j++)
                {
                    if (segGroup[i] == segGroup[j])
                    {
                        segGroup.erase(segGroup.begin() + j);
                        j--;
                    }
                }
            }
        }
    }
    void TSGeometricSolver::rTassellateSplittingSegments(std::vector<TSPolygon> &polygons,
                                                         std::vector<TSPlane> &planes,
                                                         std::vector<std::vector<TSSegment>> &segmentsGrouped)
    {
        this->m_TasselatorPtr = std::make_shared<TSTassellation>(segmentsGrouped,
                                                                 planes,
                                                                 polygons
        );
        this->m_TasselatorPtr->tassellate();
    }
    void TSGeometricSolver::rSelectFacePolygons(std::vector<TSPolygon>& polygons,
                                                std::vector<TSPolygon>& facePolygons,
                                                double tolerance,
                                                double angleToleranceDeg)
    {
        // get all the points
        std::vector<Eigen::Vector3d> tagCenters;
        for (auto& tag : this->m_Timber.getPlaneTags())
            tagCenters.push_back(tag.getCenter());

        for (uint i = 0; i < polygons.size(); i++)
        {
            // (*) get the plane associated with the polygon
            TSPlane& polyPln = polygons[i].getLinkedPlane();
            Eigen::Vector3d& polyPlnNormal = polyPln.Normal;

            // project points to the plane
            std::vector<Eigen::Vector3d> tagCentersProj;
            for (auto& ctr : tagCenters)
                tagCentersProj.push_back(polyPln.projectPoint(ctr));
            

            uint tagCounter = 0;
            for (uint j = 0; j < tagCenters.size(); j++)
            {
                // (a) check the distance between the tag's center and the plane
                double dist = polyPln.distance(tagCenters[j]);
                if (dist < tolerance)
                {
                    // (b) check the angle between the normal of the point and the plane's normal
                    double angle = TSVector::angleBetweenVectors(polyPlnNormal, 
                                                                 this->m_Timber.getPlaneTags()[j].getNormal().normalized());
                    if (angle < angleToleranceDeg)
                    {
                        // (c) check if the point is inside the polygon
                        Eigen::Vector3d ctrProj = polyPln.projectPoint(tagCenters[j]);
                        bool isInside = polygons[i].isPointInsidePolygon(ctrProj);
                        if (isInside)
                        {
                            facePolygons.push_back(polygons[i]);
                            break;
                        }
                    }
                }
            }
        }
    }

    void TSGeometricSolver::rCreateMesh()
    {
        this->rJoinPolygons(this->m_FacePolygons, this->m_MeshOutCGAL);
    }
    void TSGeometricSolver::rJoinPolygons(std::vector<TSPolygon>& facePolygons,
                                          Mesh_srf& mesh)
    {
        TSMeshHolesFiller::cvtPolygon2CGALMesh(facePolygons, mesh);

        TSMeshHolesFiller::cleanOutCGALMesh(mesh);
        TSMeshHolesFiller::fillHoles(mesh);

#ifdef TSLAM_REC_O3D_VISUAL_DEBUG
        TSMeshHolesFiller::cvtCGAL2O3dMesh(mesh, this->m_MeshOutO3d);
#endif
    }
}