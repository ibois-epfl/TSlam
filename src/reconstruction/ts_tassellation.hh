#pragma once

#include "ts_geo_util.hh"
#include <Eigen/Core>


#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Arr_non_caching_segment_traits_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/basic.h>
#include <CGAL/Arr_naive_point_location.h>

#include "arr_exact_construction_segments.hh"
#include "arr_print.hh"


typedef CGAL::Arr_naive_point_location<Arrangement>             Naive_pl;
typedef CGAL::Arr_point_location_result<Arrangement>::Type      Pl_result_type;

// typedef int                                             Number_type;
// typedef CGAL::Cartesian<Number_type>                    Kernel;
// typedef CGAL::Arr_non_caching_segment_traits_2<Kernel>  Traits;
// typedef Traits::Point_2                                 Point;    // from docu Traits_2
// typedef Traits::X_monotone_curve_2                      Segment;  // from docu Traits_2
// typedef CGAL::Arrangement_2<Traits>                     Arrangement;

namespace tslam
{
    /// TSTassellation is a simple interface parent class for tassellation of the TS.
    class TSTassellation
    {
    public:
        TSTassellation () = default;
        ~TSTassellation () = default;

    // protected:
    //     virtual void tassellate() = 0;
    
        void tassellate(std::vector<std::vector<TSSegment>> &segmentsGrouped,
                               std::vector<TSPolygon>& polygons)
        {
            // TSPolygon& p = polygons[0];                             // DEBUG
            std::vector<TSSegment>& segments = segmentsGrouped[0];  // DEBUG

            // cvt to CGAL
            std::vector<Segment> cgalSegments;
            for (auto& s : segments)
                cgalSegments.push_back(toCGALSegment(s));

            // // insert segments into arrangement by differiating between intersecting and non-intersecting segments

            
            Arrangement arr;
            Naive_pl pl(arr);
            for (auto& s : cgalSegments)
                // TODO: we need to differentiate between intersecting and not intersecting segments see 4.1.5 CGAL doc
                insert(arr, s, pl);
                // insert_non_intersecting_curve(arr, s, pl);
            print_arrangement_size(arr);



            /////////////////////////////////////////////////////////
            // Arrangement arr;
            // Naive_pl pl(arr);
            // Segment s1(Point(1, 0), Point(2, 4));
            // Segment s2(Point(5, 0), Point(5, 5));
            // Segment s3(Point(1, 0), Point(5, 3));
            // Segment s4(Point(0, 2), Point(6, 0));
            // Segment s5(Point(3, 0), Point(5, 5));
            // auto e = insert_non_intersecting_curve(arr, s1, pl);
            // insert_non_intersecting_curve(arr, s2, pl);
            // insert(arr, s3, Pl_result_type(e->source()));
            // insert(arr, s4, pl);
            // insert(arr, s5, pl);
            // print_arrangement_size(arr);
            /////////////////////////////////////////////////////////

            
            // extract boundaries of the faces
            std::vector<Face_const_handle> faces;
            std::vector<std::vector<Point>> vertices;
            for (auto f = arr.faces_begin(); f != arr.faces_end(); ++f)
            {
                if (f->is_unbounded())
                    continue;
                print_face<Arrangement>(f);
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

            // print all the vertices
            for (auto& v : vertices)
            {
                for (auto& p : v)
                    std::cout << p << " ";
                std::cout << std::endl;
            }

            // convert to TSPolygon
            for (auto& v : vertices)
            {
                TSPolygon poly;

                for (uint i = 0; i < v.size(); ++i)
                {
                    double x = CGAL::to_double(v[i].x());
                    double y = CGAL::to_double(v[i].y());
                    poly.addVertex(Eigen::Vector3d(x, y, 0));
                }

                poly.setLinkedPlane(TSPlane(0,0,1,0));
                poly.reorderClockwisePoints();

                polygons.push_back(poly);
            }


        }

    public: __always_inline  ///< converstion funcs
        // TODO: specify 2D in naming
        Segment toCGALSegment(const TSSegment& tsSegment)
        {
            double x1 = tsSegment.P1.x();
            double y1 = tsSegment.P1.y();
            double x2 = tsSegment.P2.x();
            double y2 = tsSegment.P2.y();
            return Segment(Point(x1, y1), Point(x2, y2));
        }

        // TODO: add here the polygon conversion
    };
}