#pragma once

#include "ts_timber.hh"
#include "ts_geo_util.hh"
#include "ts_rtstripe.hh"
#include "ts_tassellation.hh"

#include <Eigen/Core>


namespace tslam::Reconstruction
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
            m_Timber = TSTimber();
            m_TasselatorPtr = nullptr;

            /// default parameters
            m_RadiusSearch         = 2.0;
            m_CreaseAngleThreshold = 5.0;
            m_MinClusterSize       = 1;

            m_AABBScaleFactor      = 3.0;

            m_MaxPlnDist2Merge     = 1.0;
            m_MaxPlnAngle2Merge    = 5.0;

            m_MaxPolyTagDist       = 1.0;

            m_EPS                  = 1e-05;
            m_ShowVisualizer       = false;
        };
        ~TSGeometricSolver() = default;

        /// The main function responsible for the reconstruction
        void reconstruct();

    public:  ///< reconstruction methods
        /// (a)
        /**
         * @brief The function parse the tags following the detected creases of the timber piece. Tags are stored
         * in stripes (vector of tags) and the stripes are stored in a vector of stripes. To detect the faces we 
         * run a proximity search between the tags and we compute the angle between the normals of the tags. If the
         * angle is below a threshold we consider the tags as belonging to the same face. We also refine the stripes
         * subdivision by averaging their planes and merge similar stripes based on too close and too similar normal
         * angles planes. Once out each stripe has a plane associated passing through the extremes of the stripe and
         * defined by the average normal of all tags' normals in the stripe. The planes (without doubles) associated
         * to each stripe will be used in the next step to intersect the stripes with the AABB of the timber piece.
         * 
         */
        void rDetectFacesStripes();
            /**
             * @brief Cluster the tags based on the normal.
             * 
             * @note We employ the connected component algorithm in Cilantro library (see the reference here:
             *       https://github.com/kzampog/cilantro/blob/master/examples/connected_component_extraction.cpp)
             * @param stripesGrouped the vector of stripes
             * @param radius the radius of the search
             * @param threshDeg the threshold angle in degrees
             * @param minClusterSize the minimum cluster size
             */
            void clusterStripesByNormal(std::vector<std::shared_ptr<TSRTStripe>>& stripesGrouped,
                                        float radius, float threshDeg, int minClusterSize);
            /**
             * @brief We merge similar stripes based on their planes. The planes are similar if they are too close
             * and too similar in orientation. The merging is done by averaging the planes normals and computing the
             * new plane passing through the extremes of the stripe.
             * 
             * @param stripesGrouped the vector of stripes
             */
            void rRefineStripesByPlanes(std::vector<std::shared_ptr<TSRTStripe>>& stripesGrouped);
        
        /// (b)
        /** 
         * @brief the function intersect the edge tag's planes with a AABB and store the intersection points composing
         * the new created polygons. The similar polygons are averaged and merged into one (by averaging the 
         * intersected polygons centers get a new one per family and re-intersecting it with the AABB).
         * 
         */
        void rIntersectStripeTagPlnAABB();
        
        /// (c)
        /// Create the polysurface (list of polygons) that describes the timber solid volume
        void rCreatePolysurface();
            /**
             * @brief Obtain a vector of segments connecting the detected intersections of the AABB-generated polygons.
             * 
             * @param polygons[in] the polygons to intersect
             * @param segments[out] the segments connecting polygon's detected intersections grouped by plane
             * @param planes[out] the planes associated to the segments
             */
            void rIntersectPolygons(std::vector<TSPolygon> &polygons,
                                    std::vector<std::vector<TSSegment>> &segmentsGrouped,
                                    std::vector<TSPlane> &planes);
            /**
             * @brief Split the segments among them. This allows to obtain the polygons describing multiple faces.
             *  The found polygons still need to be selected for candidate faces of the acutal timber piece.
             * 
             * @param polygons[out] the polygons to intersect
             * @param planes[in] the planes associated to the segments
             * @param segmentsGrouped[in] the segments connecting polygon's detected intersections grouped by plane
             */
            void rTassellateSplittingSegments(std::vector<TSPolygon> &polygons,
                                              std::vector<TSPlane> &planes,
                                              std::vector<std::vector<TSSegment>> &segmentsGrouped);
            /**
             * @brief This unit selects the best candidates polygons to compose the mesh's faces. To do so, for each polygon:
             *      1. we get all the distances and get a tolerance to sift the closest tag's centers to the polygon's plane
             *      2. we check if the normals are similar to the polygon's plane by checking their angles
             *      3. we project the tag's center on the polygon's plane and check if it is inside the polygon
             * 
             * @param polygons[in] the polygons to select
             * @param facePolygons[out] the selected polygons
             * @param tolerance[in] the tolerance to select the polygons
             * @param angleToleranceDeg[in] the angle to check condition
             */
            void rSelectFacePolygons(std::vector<TSPolygon>& polygons,
                                     std::vector<TSPolygon>& facePolygons,
                                     double tolerance,
                                     double angleToleranceDeg);

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

    public: __always_inline  ///< Setters for solver parameters
        void setRadiusSearch(float radiusSearch){m_RadiusSearch = radiusSearch;}; 
        void setCreaseAngleThreshold(double creaseAngleThreshold){m_CreaseAngleThreshold = creaseAngleThreshold;};
        void setMinClusterSize(int minClusterSize){m_MinClusterSize = minClusterSize;};

        void setMaxPlnDist2Merge(double maxPlnDist){m_MaxPlnDist2Merge = maxPlnDist;};
        void setMaxPlnAngle2Merge(double maxPlnAngle){m_MaxPlnAngle2Merge = maxPlnAngle;};
        void setAABBScaleFactor(double aabbScaleFactor){m_AABBScaleFactor = aabbScaleFactor;};
        void setMaxPolyTagDist(double maxPolyDist){m_MaxPolyTagDist = maxPolyDist;};
        void setEPS(double eps){m_EPS = eps;};

        void setShowVisualizer(bool showVisualizer){m_ShowVisualizer = showVisualizer;};
        void setSolverVisualizerParams(bool drawTags = true,
                                       bool drawTagNormals = false,
                                       bool drawAabb = true,
                                       bool drawIntersectedPoly = true,
                                       bool drawSplittingSegments = false,
                                       bool drawSelectedFace = true,
                                       bool drawFinalMesh = true)
        {
            m_DrawTags = drawTags;
            m_DrawTagNormals = drawTagNormals;
            m_DrawAabb = drawAabb;
            m_DrawIntersectedPoly = drawIntersectedPoly;
            m_DrawSplittingSegments = drawSplittingSegments;
            m_DrawSelectedFace = drawSelectedFace;
            m_DrawFinalMesh = drawFinalMesh;
        };
    
    public: __always_inline  ///< Getters for solver parameters
        open3d::geometry::TriangleMesh& getMeshOut() {return this->m_MeshOut;};
    
    public: __always_inline  ///< mesh utility funcs
        /**
         * @brief Check if the geometric solver was able to produce a mesh.
         * 
         * @return true if the solver was able to produce a mesh
         * @return false if the solver was not able to produce a mesh
         */
        bool hasMesh(){return (this->m_MeshOut.HasVertices()) ? true : false; };
        /**
         * @brief Check for manifoldness and watertightness of the mesh.
         * 
         * @param mesh[in] the mesh to check
         * @return true if the mesh is manifold and watertight
         * @return false if the mesh has vertices, faces and watertight
         */
        bool checkMeshSanity(open3d::geometry::TriangleMesh& mesh)
        {
            if (mesh.HasVertices() && mesh.HasTriangles() && mesh.IsWatertight())
                return true;
            else
                return false;
        };
        /**
         * @see checkMeshSanity(open3d::geometry::TriangleMesh& mesh)
         */
        bool checkMeshSanity()
        {
            if (this->m_MeshOut.HasVertices() && this->m_MeshOut.HasTriangles() && this->m_MeshOut.IsWatertight())
                return true;
            else
                return false;
        };

    public:  __always_inline  ///< clean out memory func
        /// Function to clean all the members and memory linked to the solver
        void clean()
        {
            this->m_Timber.clean();
            this->m_TasselatorPtr = nullptr;

            this->m_PlnAABBPolygons.clear();
            this->m_SplitSegmentsPlanes.clear();
            this->m_SplitSegmentsGrouped.clear();
            this->m_SplitPolygons.clear();
            this->m_FacePolygons.clear();

            this->m_MeshOut.Clear();
            this->m_MeshOutXAC.Clear();
        };

    private:  ///< Visualizer
        /// visualize the results of the timber reconstruction
        void visualize(bool showVisualizer,
                       bool drawTags,
                       bool drawTagNormals,
                       bool drawAabb,
                       bool drawIntersectedPoly,
                       bool drawSplittingSegments,
                       bool drawSelectedFace,
                       bool drawFinalMesh);
        /// Show the visulier
        bool m_ShowVisualizer;
        /// Show the tags as wireframe
        bool m_DrawTags;
        /// Show the tags' normals
        bool m_DrawTagNormals;
        /// Show the Axis-Aligned Bounding Box (AABB) of the timber element
        bool m_DrawAabb;
        /// Draw the intersect polygons with the AABB
        bool m_DrawIntersectedPoly;
        /// Show the splitting segments for each intersected polygon
        bool m_DrawSplittingSegments;
        /// Show the selected face polygons
        bool m_DrawSelectedFace;
        /// Show the timber volume after merging into a mesh
        bool m_DrawFinalMesh;

    public: __always_inline  ///< getters
        /// Get the timber element
        TSTimber& getTimber(){return this->m_Timber;};
        /// Get the tassellator
        std::shared_ptr<TSTassellation>& getTasselator(){return this->m_TasselatorPtr;};

        /// Get number of tags
        int getNbrTags(){return this->m_Timber.getNbrTags();};
        /// Get number of intersected polygons with the AABB
        int getNbrPlnAABBPolygons(){return this->m_PlnAABBPolygons.size();};
        /// Get number of splitting segments
        int getNbrSplitSegments()
        {
            int nbrSplitSegments = 0;
            for (auto& splitSegments : this->m_SplitSegmentsGrouped)
                nbrSplitSegments += splitSegments.size();
            return nbrSplitSegments;
        };
        /// Get number of face polygons
        int getNbrFacePolygons(){return this->m_FacePolygons.size();};
        /// Get number of vertices of the final mesh
        int getNbrMeshVertices(){return this->m_MeshOut.vertices_.size();};

    private:  ///< Solver components
        /// The timber component of the geometric solver contains all the tags and tag stripes
        TSTimber m_Timber;
        /// The tassellotor component object is a plugin responsible for the tassellation of spliiting segments
        std::shared_ptr<TSTassellation> m_TasselatorPtr;
    
    private:  ///< Solver parameters for user's tuning
        /// Radius search for the normal clustering of the tags
        float m_RadiusSearch;
        /// The threshold for detection of crease's angle (the smaller the more creases will be detected)
        double m_CreaseAngleThreshold;
        /// Minimum cluster possible of tags
        int m_MinClusterSize;

        /// The scale factor for scaleing up the AABB of the timber element
        double m_AABBScaleFactor;
        /// The maximal distance between a polygon and a tag to be considered as a candidate face
        double m_MaxPolyTagDist;
        /// The maximal distance between planes of stripes to be eligible for merging
        double m_MaxPlnDist2Merge;
        /// The maximal angle difference in degs between two planes'normals' angle to be eligible for merging
        double m_MaxPlnAngle2Merge;
        /// The tolerance for all the computation (e.g. for the intersections)
        double m_EPS;

    private:  ///< Solver internal variables
        /// Vector of polygons issued of tags' planes-AABB intersections
        std::vector<TSPolygon> m_PlnAABBPolygons;
        /// Vector to store the planes on which the splitting segments are located
        std::vector<TSPlane> m_SplitSegmentsPlanes;
        /// Vector of vectors of splitting segments grouped by planes
        std::vector<std::vector<TSSegment>> m_SplitSegmentsGrouped;
        /// Vector of split polygons
        std::vector<TSPolygon> m_SplitPolygons;
        /// Face polygons to create the mesh
        std::vector<TSPolygon> m_FacePolygons;
        /// The timber mesh in format PLY
        open3d::geometry::TriangleMesh m_MeshOut;
        // TODO: to implement the XAC format and storage for importing on 3d modeler
        /// The timber mesh in format XAC
        open3d::geometry::TriangleMesh m_MeshOutXAC;

    private:  ///< Profiler  // TODO: this needs to be implemented (mayybe for the all tslam)
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