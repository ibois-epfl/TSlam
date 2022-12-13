#pragma once

#include "ts_timber.hh"
#include "ts_geo_util.hh"

#include <Eigen/Core>

// FIXME: the averaging of the planes is not accurate enough

namespace tslam
{
    /**
     * @brief TSGeometricSolver class responsible for reconstructing a mesh of the timber from its tags
     * 
     */
    class TSGeometricSolver
    {
    public:
        TSGeometricSolver() 
        {
            m_AABBScaleFactor=2.0;
            m_CreaseAngleThreshold=10.0;
            m_MinPolyDist=3.0;
        };
        ~TSGeometricSolver() = default;

        // TODO: update solver steps
        /** 
         * @brief Ths function reconstruct a mesh from the TSlam map composed by Tags.
         * 0. remove duplicate tags from map
         * 1. detect the creases of the timber piece by proximity search and angle 
         * difference between tags'normals
         * 2. intersect selected planes with AABB
         * 3. merge similar planes
         * 4. intersect planes(intersected polygons) with each other and generate new polygons
         * 5. keep only the polygons with tags' corner points inside them
         * 6. join the polygons into a mesh
         * 7. check mesh for watertightness and manifoldness
         * (8. get only the contours and not the inner polygons)
         * 
        */
        void reconstruct();

    private:  ///< reconstruction methods
        /// (a)
        // FIXME: this function should be a timber.hh's function
        /**
         * @brief The function detect the tags creases of the timber piece. It builds a ktree of the tags and by
         * k-nearest neighbor search it finds the nearest tags to each tag. Then it computes the angle between
         * the normals of the tags and if the angle is smaller than a threshold it is considered a crease.
         * 
         */
        void rDetectCreasesTags();
            /** 
             * @brief It computes the angle between two vectors
             * 
             * @param v1 the first vector
             * @param v2 the second vector
             * @return double the angle between the two vectors in degrees
             */
            double rAngleBetweenVectors(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);
        
        /// (b)
        /** 
         * @brief the function intersect a plane with a AABB and store the intersection points.
         * 
         */
        void rIntersectTagPlnAABB();
            // TODO: this function should be a plane member function
            /** 
             * @brief It checks if there is intersection between a ray and a plane following the:
             * Plane: ax+by+cz=d
             * Ray: P(t) = P0 + t * D
             * Intersection: P(t) = P0 + t * D = (x,y,z) = (a,b,c) * t = (a,b,c) * (d - (a*x+b*y+c*z)) / (a*a+b*b+c*c)
             * t = (d - (a*x+b*y+c*z)) / (a*a+b*b+c*c)
             * 
             * @see the function is modified from: https://asawicki.info/news_1428_finding_polygon_of_plane-aabb_intersection
             * 
             * @param RayOrig[out] the origin of the ray
             * @param RayDir[out] the direction of the ray
             * @param Plane[out] the plane to check the intersection with
             * @param OutT[in] the distance from the ray origin to the intersection point
             * @param OutVD[in] the distance from the ray origin to the plane
             * 
             * @return true if there is intersection
             * @return false if there is no intersection
             */
            bool rRay2PlaneIntersection(const Eigen::Vector3d &RayOrig,
                                        const Eigen::Vector3d &RayDir,
                                        const TSPlane &Plane,
                                        float *OutT,
                                        float *OutVD);
            // TODO: this function should be a plane member function
            /** 
             * @brief It computes the intersection points between a plane and an AABB
             * 
             * @see the function is modified from: https://asawicki.info/news_1428_finding_polygon_of_plane-aabb_intersection
             * 
             * @param plane[out] the plane to check the intersection with
             * @param aabb_min[out] the minimum point of the AABB
             * @param aabb_max[out] the maximum point of the AABB
             * @param out_points[in] the intersection points
             * @param out_point_count[in] the number of intersection points (min:3, max: 6)
             */
            void rPlane2AABBSegmentIntersect(const TSPlane &plane,
                                            const Eigen::Vector3d &aabb_min,  // TODO: refactor snakecase to camel
                                            const Eigen::Vector3d &aabb_max,
                                            Eigen::Vector3d* out_points,
                                            unsigned &out_point_count);
        
        /// (c)
        /**
         * @brief The function mean the previous similar polygons'planes and recompute the
         * intersection of the mean planes with the AABB to obtain new polygons.
         * 
         */
        void rIntersectMeanPolygonPlnAABB();
            // TODO: this function should be a polygon member function
            /**
             * @brief The function merge polygons/planes that are similar in orientation and close to 
             * each  other. It builds a ktree of the polygons centers and group the "close" polygons.
             * A new center and a ew normal are computed and a new plane is created.
             * 
             */
            void rMeanPolygonPlanes();

        /// (d)
        /**
         * @brief The function intersect the polygons with each other and generate new polygons.
         * It keeps only those polygons that have at least one corner point inside them.
         * 
         */
        void rIntersectPolygons();
            // /**
            //  * @brief Check if an intersection exist between two segments
            //  * 
            //  * @param p1 the first point of the first segment
            //  * @param p2 the second point of the first segment
            //  * @param p3 the first point of the second segment
            //  * @param p4 the second point of the second segment
            //  * @param intersectPt[out] the intersection point
            //  * 
            //  * @return true if there is intersection
            //  * @return false if there is no intersection
            //  */
            // bool rSegment2SegmentIntersect(TSSegment& segA, 
            //                                TSSegment& segB,
            //                                Eigen::Vector3d* intersectPt);
            /**
             * @brief It splits a polygon with a segment and give back the two new polygons.
             * 
             * @param poly[in] the polygon to split
             * @param seg[in] the splitter segment
             * @param splitPoly[out] the two new polygons after the split
             * @return true if the split is successful
             * @return false if the split is not successful or there is no intersection
             */
            bool rPolygon2SegmentSplit(TSPolygon& poly, 
                                       TSSegment& seg, 
                                       std::tuple<TSPolygon, TSPolygon>& splitPolys);
            // /**
            //  * @brief The function intersect two polygons and store the intersection points.
            //  * 
            //  * @param polyA[in] the first polygon
            //  * @param polyB[in] the second polygon
            //  * @param outPtsCount[out] the number of intersection points
            //  * @param outSplitPolygons[out] the two new polygons after the intersection
            //  */
            // // FIXME: on hold
            // // TODO: this function should be a polygon member function
            // bool rPolygon2PolygonIntersect(TSPolygon& polyA, 
            //                                TSPolygon& polyB,
            //                                std::tuple<TSPolygon, TSPolygon>& outSplitPolygons);

        /// (e)
        /**
         * @brief It joins the polygons and create a new mesh of the timber object.
         * 
         */
        void rCreateMesh();
            /**
             * @brief It joins all the polygons in a new mesh.
             * 
             */
            void rJoinPolygons();
            /**
             * @brief Check for manifoldness and watertightness of the mesh.
             * 
             */
            void rCheckMeshSanity();

    public: __always_inline  ///< Setters for solver parameters
        void setTimber(std::shared_ptr<TSTimber> timber){m_Timber = timber; check4PlaneTags();};
        void setCreaseAngleThreshold(double crease_angle_threshold){m_CreaseAngleThreshold = crease_angle_threshold;};
        void setMinPolyDist(double min_poly_dist){m_MinPolyDist = min_poly_dist;};

    private:  ///< utility funcs
            /** 
             * @brief check4PlaneTags checks if the timber object has plane tags
             * 
             * @return true if the timber object has plane tags
             * @return false if the timber object has no plane tags
             */
            bool check4PlaneTags();

    private:  ///< I/O funcs
            /**
             * @brief Export the timber mesh to a .ply file locally.
             * 
             */
            void exportMesh2PLY();  // TODO: implement
    private:  ///< Solver parameters
        /// The timber element to reconstruct
        std::shared_ptr<tslam::TSTimber> m_Timber;
        /// The threshold for detection of crease's angle (the smaller the more creases will be detected)
        double m_CreaseAngleThreshold;
        /// The scale factor for scaleing up the AABB of the timber element
        double m_AABBScaleFactor;
        /// The minimum distance between two polygons' centers to be merged
        double m_MinPolyDist;

    private:  ///< Solver internal variables
        /// Vector of polygons issued of tags' planes-AABB intersections
        std::vector<TSPolygon> m_PlnAABBPolygons;
        /// Vector of merged close and similar polygons
        std::vector<TSPolygon> m_MergedPolygons;

    private:  ///< Profiler
#ifdef TSLAM_REC_PROFILER
        inline void timeStart(const char* msg)
        {
            m_time_start = std::chrono::high_resolution_clock::now();
            m_time_msg = msg;
        };
        inline void timeEnd()
        {
            m_time_end = std::chrono::high_resolution_clock::now();
            m_time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(m_time_end - m_time_start);
            std::cout << this->m_time_msg << " : " << m_time_elapsed.count() << " ms" << std::endl;
        };

        std::chrono::time_point<std::chrono::high_resolution_clock> m_time_start;
        std::chrono::time_point<std::chrono::high_resolution_clock> m_time_end;
        const char* m_time_msg;
        std::chrono::milliseconds m_time_elapsed;
#else
        inline void timeStart(const char* msg){};
        inline void timeEnd(){};
#endif
    };
}