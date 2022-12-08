#pragma once

#include "ts_timber.hh"

#include <Eigen/Core>

// TODO: add I/O functions to export mesh

namespace tslam
{
    /**
     * @brief TSPolygon is an utility struct to store a polygon
     * 
     */
    struct TSPolygon : public TSObject
    {
        TSPolygon() = default;
        TSPolygon(std::vector<Eigen::Vector3d> points, TSTPlane linkedPlane)
            : m_Points(points), m_LinkedPlane(linkedPlane)
        {
            this->compute();
        };
        ~TSPolygon() = default;

    private: __always_inline
        void compute() override 
        {
            computeCenter();
        };
        void computeCenter()
        {
            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (auto p : m_Points)
                center += p;
            center /= m_Points.size();
            m_Center = center;
        };

    private:
        std::vector<Eigen::Vector3d> m_Points;
        Eigen::Vector3d m_Center;
        /// The plane on which the polygon lies
        tslam::TSTPlane m_LinkedPlane;
    
    public: __always_inline
        void setPoints(std::vector<Eigen::Vector3d> points) {m_Points = points; compute(); };
        void setLinkedPlane(TSTPlane linkedPlane) {m_LinkedPlane = linkedPlane; };

    public:__always_inline
        std::vector<Eigen::Vector3d>& getPoints() {return m_Points; };
        uint getNumPoints() {return m_Points.size(); };
        Eigen::Vector3d& getPoint(uint i) {return m_Points[i]; };
        Eigen::Vector3d& getCenter() {return m_Center; };
        Eigen::Vector3d getNormal() {return m_LinkedPlane.getNormal(); };
        /**
         * @brief Get the Linked Plane object
         * 
         * @return tslam::TSTPlane& it returns the plane on which the polygon lies
         */
        tslam::TSTPlane& getLinkedPlane() {return m_LinkedPlane; };
    };

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
        
        /// (c)
        /**
         * @brief The function mean the previous similar polygons'planes and recompute the
         * intersection of the mean planes with the AABB to obtain new polygons.
         * 
         */
        void rIntersectMeanPolygonPlnAABB();
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
            /**
             * @brief The function intersect two polygons and store the intersection points.
             * 
             * @param poly1 the first polygon
             * @param poly2 the second polygon
             * @param out_points the intersection points
             * @param out_point_count the number of intersection points
             */
            void rPoly2PolyIntersect(const TSPolygon& poly1, 
                                     const TSPolygon& poly2,
                                     Eigen::Vector3d* out_points,  //TODO: delete manually
                                     unsigned& out_point_count);  //TODO: delete manually
            /**
             * @brief It checks if a point is inside a polygon.
             * 
             * @param point the point to check
             * @param poly the polygon to check
             * @return true if the point is inside the polygon
             * @return false if the point is outside the polygon
             */
            bool rPointInPoly(const Eigen::Vector3d& point, const TSPolygon& poly);

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