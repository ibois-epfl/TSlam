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

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel       Kernel;
typedef Kernel::FT                                              Number_type;
      
typedef CGAL::Arr_segment_traits_2<Kernel>                      Traits;
typedef Traits::Point_2                                         Point;
typedef Traits::X_monotone_curve_2                              Segment;
      
typedef CGAL::Arrangement_2<Traits>                             Arrangement;
typedef Arrangement::Vertex_handle                              Vertex_handle;
typedef Arrangement::Halfedge_handle                            Halfedge_handle;
typedef Arrangement::Face_handle                                Face_handle;
typedef Arrangement::Vertex_const_handle                        Vertex_const_handle;
typedef Arrangement::Halfedge_const_handle                      Halfedge_const_handle;
typedef Arrangement::Face_const_handle                          Face_const_handle;

typedef CGAL::Arr_naive_point_location<Arrangement>             Naive_pl;
typedef CGAL::Arr_point_location_result<Arrangement>::Type      Pl_result_type;

namespace tslam::Reconstruction
{
    /// TSTassellation is a simple interface parent class for tassellation of the TS.
    class TSTassellation
    {
    public:
        TSTassellation() = default;
        TSTassellation(std::vector<std::vector<TSSegment>>& segmentsGrouped,
                       std::vector<TSPlane>& planes,
                       std::vector<TSPolygon>& polygons);
        ~TSTassellation() = default;

    public:  ///< tassellation interface
        /// Tassellate the intersecting segments to obtain the intersecting areas as polygons.
        void tassellate();

    private:  ///< tassellate functions
        /**
         * @brief Bring all the segments to the XY plane because CGAL segmentation is only 2D.
         * 
         * @param segmentsGrouped the splitting segments grouped
         * @param planes the target planes to retransform back
         * @param invMats the inverse matrices to retransform back
         */
        void transformToXYPlane(std::vector<std::vector<TSSegment>>& segmentsGrouped,
                                std::vector<TSPlane>& planes,
                                std::vector<Eigen::Matrix4d>& invMats);
        /**
         * @brief Tassellate the intersecting segments to obtain the intersecting areas as polygons.
         * 
         * @param segmentsGrouped[in] the splitting segments grouped
         * @param segPolygonsGrouped[out] the output polygons vector
         */
        void runSegmentation2D(std::vector<std::vector<TSSegment>>& segmentsGrouped,
                               std::vector<std::vector<TSPolygon>>& segPolygonsGrouped);
        /**
         * @brief Invert the transform and bring back the segmented polygons to the origina volume.
         * 
         * @param segPolygonsGrouped[in] the segmented polygons
         * @param segmentsGrouped[in] the grouped splitting segments to retransform back
         * @param planes[in] the target planes to relink to the polygons
         * @param invMats[in] the inverse transform matrices
         * @param polygons[out] the final retransformed polygons
         */
        void transformToOriginalPlane(std::vector<std::vector<TSPolygon>>& segPolygonsGrouped,
                                      std::vector<std::vector<TSSegment>>& segmentsGrouped,
                                      std::vector<TSPlane>& planes,
                                      std::vector<Eigen::Matrix4d>& invMats,
                                      std::vector<TSPolygon>& polygons);
    
    private:  ///< converstion funcs
        /**
         * @brief Convert a TSSegment to a CGAL Segment
         * 
         * @param tsSegment the TSSegment
         * @return Segment the CGAL Segment
         */
        Segment toCGALSegment(const TSSegment& tsSegment);
        /**
         * @brief Convert a CGAL group of vertices to a TSPolygon
         * 
         * @param cgalVertices the CGAL vertices
         * 
         * @return TSPolygon the TSPolygon
         */
        TSPolygon toTSPolygon(const std::vector<Point>& cgalVertices);
    
    public: __always_inline  ///< clean func
        /// Clean the tassellation memories and data attached
        void clean()
        {
            this->m_PolygonsGrouped.clear();
            this->m_InvMats.clear();
        };
    
    public: ///< set funcs
        /// Set the reference to the splitting segments grouped
        void setSegmentsGroupedRef(std::vector<std::vector<TSSegment>>& segmentsGrouped);
        /// Set the reference to the target planes
        void setPlanesRef(std::vector<TSPlane>& planes);
        /// Set the reference to the output polygons
        void setPolygonsRef(std::vector<TSPolygon>& polygons);

    public:  ///< reference members
        /// the reference to the splitting segments grouped
        std::vector<std::vector<TSSegment>>& SegmentsGrouped_ref;
        /// the reference to the target planes
        std::vector<TSPlane>& Planes_ref;
        /// the reference to the output polygons
        std::vector<TSPolygon>& Polygons_ref;

    private:  ///< internal members
        /// the splitting segments grouped
        std::vector<std::vector<TSPolygon>> m_PolygonsGrouped;
        /// the inverse transform matrices
        std::vector<Eigen::Matrix4d> m_InvMats;
    };
}