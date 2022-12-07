#pragma once

#include "ts_timber.hh"

#include <Eigen/Core>


namespace tslam
{
    class TSGeometricSolver
    {
    public:
        TSGeometricSolver() 
        {
            m_AABBScaleFactor=2.0;
        };
        ~TSGeometricSolver() = default;

        /** 
         * @brief Ths function reconstruct a mesh from the TSlam map composed by Tags.
         * 0. remove duplicate tags from map
         * 1. detect the creases of the timber piece by proximity search and angle 
         * difference between tags'normals
         * 2. intersect selected planes with AABB
         * 3. intersect planes(intersected polygons) with each other and generate new polygons
         * 4. keep only the polygons with tags' corner points inside them
         * 5. join the polygons into a mesh
         * 6. check mesh for watertightness and manifoldness
         * (7. get only the contours and not the inner polygons)
         * 
        */
        void reconstruct();

    private:
        /// the function intersect a plane with a AABB and store the intersection points.
        void rIntersectTagPlnAABB();
            /** 
             * @brief It checks if there is intersection between a ray and a plane following the:
             * Plane: ax+by+cz=d
             * Ray: P(t) = P0 + t * D
             * Intersection: P(t) = P0 + t * D = (x,y,z) = (a,b,c) * t = (a,b,c) * (d - (a*x+b*y+c*z)) / (a*a+b*b+c*c)
             * t = (d - (a*x+b*y+c*z)) / (a*a+b*b+c*c)
             * 
             * @see the function is modified from: https://asawicki.info/news_1428_finding_polygon_of_plane-aabb_intersection
             * 
             * @param RayOrig the origin of the ray
             * @param RayDir the direction of the ray
             * @param Plane the plane to check the intersection with
             * @param OutT the distance from the ray origin to the intersection point
             * @param OutVD the distance from the ray origin to the plane
             * 
             * @return true if there is intersection
             * @return false if there is no intersection
             */
            bool rRay2PlaneIntersection(const Eigen::Vector3d &RayOrig,
                                        const Eigen::Vector3d &RayDir,
                                        const TSTPlane &Plane,
                                        float *OutT,
                                        float *OutVD);
            /** 
             * @brief It computes the intersection points between a plane and an AABB
             * 
             * @see the function is modified from: https://asawicki.info/news_1428_finding_polygon_of_plane-aabb_intersection
             * 
             * @param plane the plane to check the intersection with
             * @param aabb_min the minimum point of the AABB
             * @param aabb_max the maximum point of the AABB
             * @param out_points the intersection points
             * @param out_point_count the number of intersection points (min:3, max: 6)
             */
            void rPlane2AABBSegmentIntersect(const TSTPlane &plane,
                                            const Eigen::Vector3d &aabb_min, 
                                            const Eigen::Vector3d &aabb_max,
                                            Eigen::Vector3d* out_points,
                                            unsigned &out_point_count);
            /** 
             * @brief It reorder the intersection points in a clockwise order and store them in a variable member
             * 
             * @see the function is modified from: https://asawicki.info/news_1428_finding_polygon_of_plane-aabb_intersection
             * 
             * @param points the intersection points
             * @param point_count the number of intersection points
             * @param plane the plane to check the intersection with
             */
            void rSortIntersectionPoints(Eigen::Vector3d* points, 
                                        unsigned point_count,
                                        const TSTPlane& plane);

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

    public: __always_inline
        void setTimber(std::shared_ptr<TSTimber> timber){m_Timber = timber; check4PlaneTags();};
        void setCreaseAngleThreshold(double crease_angle_threshold){m_CreaseAngleThreshold = crease_angle_threshold;};

    private:
            /** 
             * @brief check4PlaneTags checks if the timber object has plane tags
             * 
             * @return true if the timber object has plane tags
             * @return false if the timber object has no plane tags
             */
            bool check4PlaneTags();
    private:
        /// The timber element to reconstruct
        std::shared_ptr<tslam::TSTimber> m_Timber;
        /// The threshold for detection of crease's angle (the smaller the more creases will be detected)
        double m_CreaseAngleThreshold;
        /// The scale factor for scaleing up the AABB of the timber element
        double m_AABBScaleFactor;
        /// Vector of polygon's points of intersection between the planes and the AABB of the timber element
        std::vector<std::vector<Eigen::Vector3d>> m_IntersectPlnAABBPts;

    public:
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
#else
        inline void timeStart(const char* msg){};
        inline void timeEnd(){};
#endif

    private:
#ifdef TSLAM_REC_PROFILER
        std::chrono::time_point<std::chrono::high_resolution_clock> m_time_start;
        std::chrono::time_point<std::chrono::high_resolution_clock> m_time_end;
        const char* m_time_msg;
        std::chrono::milliseconds m_time_elapsed;
#endif
    };
}