#pragma once

#include "ts_timber.hh"
#include "ts_geo_util.hh"
#include "ts_rtstripe.hh"

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
            m_CreaseAngleThreshold=10.0;
            m_MinPolyDist=3.0;
            m_AABBScaleFactor=2.0;
            m_MaxPolyTagDist=0.5;
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
         * @brief The function seperate the tags in stripes belonging to the same face by detecting faces or creases.
         * 
         */
        // FIXME: rename me appropiately, e.g. detectCreases (sub-function: parse tags in stripes)
        void rDetectCreasesTags();
        
        /// (b)
        /** 
         * @brief the function intersect the edge tag's planes with a AABB and store the intersection points composing
         * the new created polygons. The similar polygons are averaged and merged into one (by averaging the 
         * intersected polygons centers get a new one per family and re-intersecting it with the AABB).
         * 
         */
        void rIntersectStripeTagPlnAABB();
            /**
             * @brief The function merge polygons/planes that are similar in orientation and close to 
             * each  other. It builds a ktree of the polygons centers and group the "close" polygons' centers.
             * A new center and a new normal are computed and a new plane is created.
             * 
             */
            void rMeanPolygonPlanes();
            /**
             * @brief The function mean the previous similar polygons'planes and recompute the
             * intersection of the mean planes with the AABB to obtain new polygons.
             * 
             */
            void rIntersectMeanPolygonPlnAABB();
        
        /// (c)
        /// Create the polysurface (list of polygons) that describes the timber solid volume
        void rCreatePolysurface();
            /**
             * @brief Obtain a vector of segments connecting the detected intersections of the AABB-generated polygons.
             * 
             * @param polygons[in] the polygons to intersect
             * @param segments[out] the segments connecting polygon's detected intersections
             */
            void rIntersectPolygons(std::vector<TSPolygon> &polygons,
                                    std::vector<TSSegment> &segments);
            /**
             * @brief Split the polygons into smaller ones with the intersecting segments.
             * 
             * @param polygons[in] the polygons to split
             * @param splitPolygons[out] the split polygons
             * @param segments[in] the segments to split the polygons
             */
            void rSplitPolygons(std::vector<TSPolygon>& polygons,
                                std::vector<TSPolygon>& splitPolygons,
                                std::vector<TSSegment>& segments);
            /**
             * @brief This unit selects the best candidates polygons to compose the mesh's faces.
            To do this selection we do the following:
            for each set of split polygons:
                for each polygon:
                    for each tag:
                        get the distance to the polygon's plane
                        if distance is less than a threshold:
                            project tag center to polygon's plane
                            if tag is inside polygon:
                                add polygon to face
                                break
            * 
            * @param polygons[in] the polygons to select
            * @param splitPolygons[out] the selected polygons
            * @param tolerance[in] the tolerance to select the polygons
            */
            void rSelectFacePolygons(std::vector<TSPolygon>& polygons,
                                    std::vector<TSPolygon>& facePolygons,
                                    double tolerance);

        /// (d)
        /**
         * @brief Create the mesh out of the candidate polygon faces. The mesh is created by joining the polygons.
         * *** The mesh has no normals/orientation ***
         * 
         */
        void rCreateMesh();
            /**
             * @brief It joins all the polygons in a new o3d triangle mesh.
             * 
             * @param facePolygons[in] the polygons to join
             * @param mesh[out] the mesh to create
             */
            void rJoinPolygons(std::vector<TSPolygon>& facePolygons,
                               open3d::geometry::TriangleMesh& mesh);
            /**
             * @brief Check for manifoldness and watertightness of the mesh.
             * 
             * @param mesh[in] the mesh to check
             * @return true if the mesh is manifold and watertight
             * @return false if the mesh is not manifold or watertight
             */
            bool rCheckMeshSanity(open3d::geometry::TriangleMesh& mesh);

    public: __always_inline  ///< Setters for solver parameters
        void setTimber(std::shared_ptr<TSTimber> timber){m_Timber = timber; check4PlaneTags();};
        void setCreaseAngleThreshold(double crease_angle_threshold){m_CreaseAngleThreshold = crease_angle_threshold;};
        void setMinPolyDist(double min_poly_dist){m_MinPolyDist = min_poly_dist;};  // FIXME: to erase
        void setMaxPlnDist2Merge(double max_pln_dist){m_MaxPlnDist2Merge = max_pln_dist;};
        void setMaxPlnAngle2Merge(double max_pln_angle){m_MaxPlnAngle2Merge = max_pln_angle;};
        void setAABBScaleFactor(double aabb_scale_factor){m_AABBScaleFactor = aabb_scale_factor;};
        void setMaxPolyTagDist(double max_poly_dist){m_MaxPolyTagDist = max_poly_dist;};

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
    private:  ///< Solver parameters for user's tuning
        /// The timber element to reconstruct
        std::shared_ptr<tslam::TSTimber> m_Timber;
        /// The threshold for detection of crease's angle (the smaller the more creases will be detected)
        double m_CreaseAngleThreshold;
        /// The scale factor for scaleing up the AABB of the timber element
        double m_AABBScaleFactor;
        /// The minimum distance between two polygons' centers to be merged
        double m_MinPolyDist;  //FIXME: to erase, no more needed
        /// The maximal distance between a polygon and a tag to be considered as a candidate face in meters (0.03 ~3cm)
        double m_MaxPolyTagDist;
        /// The maximal distance between planes of stripes to be eligible for merging
        double m_MaxPlnDist2Merge;
        /// The maximal angle difference in radians between two planes' normals to be eligible for merging
        double m_MaxPlnAngle2Merge;

    protected:  ///< Solver internal variables
        /// Vector of polygons issued of tags' planes-AABB intersections
        std::vector<TSPolygon> m_PlnAABBPolygons;
        /// Vector of merged close and similar polygons
        std::vector<TSPolygon> m_MergedPolygons;
        /// Vector of splitting segments out of main polygons' intersection
        std::vector<TSSegment> m_SplitSegments;
        /// Vector of split polygons
        std::vector<TSPolygon> m_SplitPolygons;
        /// Face polygons to create the mesh
        std::vector<TSPolygon> m_FacePolygons;
        /// The timber mesh
        open3d::geometry::TriangleMesh m_MeshOut;

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