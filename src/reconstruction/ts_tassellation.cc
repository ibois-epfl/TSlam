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
#include "ts_tassellation.hh"

namespace tslam::Reconstruction
{
    TSTassellation::TSTassellation(std::vector<std::vector<TSSegment>>& segmentsGrouped,
                                   std::vector<TSPlane>& planes,
                                   std::vector<TSPolygon>& polygons)
        : SegmentsGrouped_ref(segmentsGrouped),
          Planes_ref(planes),
          Polygons_ref(polygons)
    {}

    void TSTassellation::tassellate()
    {
        // (A) transform to plane-2-XYplane the segments
        this->transformToXYPlane(this->SegmentsGrouped_ref,
                                 this->Planes_ref,
                                 this->m_InvMats);

        // (B) run the segmentation2d of segment intersections from CGAL
        this->runSegmentation2D(this->SegmentsGrouped_ref,
                                this->m_PolygonsGrouped);

        // (C) transform XYplane-2-plane back to 3D segments & polygons
        this->transformToOriginalPlane(this->m_PolygonsGrouped,
                                       this->SegmentsGrouped_ref,
                                       this->Planes_ref,
                                       this->m_InvMats,
                                       this->Polygons_ref);
    }

    void TSTassellation::transformToXYPlane(std::vector<std::vector<TSSegment>> &segmentsGrouped,
                                            std::vector<TSPlane>& planes,
                                            std::vector<Eigen::Matrix4d>& invMats)
    {
        for (uint i = 0; i < segmentsGrouped.size(); i++)
        {
            // get the 4x4 matrix plane-to-XYplane
            auto& segs = segmentsGrouped[i];
            TSPlane& planeA = planes[i];
            Eigen::Matrix4d mat = TSPlane::getPlane2XYPlaneRotation(planeA);

            // bring the segments to XY plane
            for (auto& seg : segs)
                seg.transform(mat);
            
            // store the inverse matrix for later use
            Eigen::Matrix4d invMat = mat.inverse();
            invMats.push_back(invMat);
        }
    }
    
    void TSTassellation::runSegmentation2D(std::vector<std::vector<TSSegment>>& segmentsGrouped,
                                           std::vector<std::vector<TSPolygon>>& segPolygonsGrouped)
    {
        for (auto& segments : segmentsGrouped)
        {
            // cvt segments to CGAL format
            std::vector<Segment> cgalSegments;
            for (auto& s : segments)
                cgalSegments.push_back(this->toCGALSegment(s));
            
            // build arrangements2d CGAL
            Arrangement arr;
            Naive_pl pl(arr);
            for (auto& s : cgalSegments)
                insert(arr, s, pl);

            // extract boundaries of the faces
            std::vector<Face_const_handle> faces;
            std::vector<std::vector<Point>> vertices;
            for (auto f = arr.faces_begin(); f != arr.faces_end(); ++f)
            {
                if (f->is_unbounded())
                    continue;
                faces.push_back(f);

                // extract the vertices
                std::vector<Point> v;
                auto c = f->outer_ccb();
                do
                {
                    v.push_back(c->source()->point());
                    c = c->next();
                } while (c != f->outer_ccb());
                vertices.push_back(v);
            }

            // convert to TSPolygon
            std::vector<TSPolygon> segPolygons;
            for (auto& v : vertices)
            {
                TSPolygon poly = this->toTSPolygon(v);
                segPolygons.push_back(poly);
            }

            segPolygonsGrouped.push_back(segPolygons);
        }
    }
    
    void TSTassellation::transformToOriginalPlane(std::vector<std::vector<TSPolygon>>& segPolygonsGrouped,
                           std::vector<std::vector<TSSegment>>& segmentsGrouped,
                           std::vector<TSPlane>& planes,
                           std::vector<Eigen::Matrix4d>& invMats,
                           std::vector<TSPolygon>& polygons)
    {
        // transform back to original plane the new tassellated polygons
        for (uint i = 0; i < segPolygonsGrouped.size(); i++)
        {
            for (uint j = 0; j < segPolygonsGrouped[i].size(); j++)
            {
                segPolygonsGrouped[i][j].transform(invMats[i]);
                segPolygonsGrouped[i][j].setLinkedPlane(planes[i]);

                polygons.push_back(segPolygonsGrouped[i][j]);
            }
        }

        // transform back to original plane the original segments
        for (uint i = 0; i < segmentsGrouped.size(); i++)
            for (uint j = 0; j < segmentsGrouped[i].size(); j++)
                segmentsGrouped[i][j].transform(invMats[i]);
    }
    
    Segment TSTassellation::toCGALSegment(const TSSegment& tsSegment)
    {
        double x1 = tsSegment.P1.x();
        double y1 = tsSegment.P1.y();
        double x2 = tsSegment.P2.x();
        double y2 = tsSegment.P2.y();

        Segment seg = Segment(Point(x1, y1), Point(x2, y2));
        return seg;
    }

    TSPolygon TSTassellation::toTSPolygon(const std::vector<Point>& cgalVertices)
    {
        TSPolygon poly;

        for (uint i = 0; i < cgalVertices.size(); ++i)
        {
            double x = CGAL::to_double(cgalVertices[i].x());
            double y = CGAL::to_double(cgalVertices[i].y());
            poly.addVertex(Eigen::Vector3d(x, y, 0));
        }

        poly.setLinkedPlane(TSPlane(0,0,1,0));
        poly.reorderClockwisePoints();

        return poly;
    }
}