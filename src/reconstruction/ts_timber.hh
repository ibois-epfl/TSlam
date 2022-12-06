#pragma once

#include "ts_rtag.hh"
#include <open3d/Open3D.h>
#include <Eigen/Core>

//TODO: Tag and Timber classes should inherit from a parent class with interface function compute

namespace tslam
{
    /**
     * @brief TSTimber represents the timber element to be processed. It containes the
     * tags' geometrical and metdata info as well as the reconstruction.
     * 
     */
    class TSTimber : public TSObject
    {
    public:
        TSTimber() {m_RTags = {}; };
        TSTimber(std::vector<tslam::TSRTag> planeTags);
        ~TSTimber() = default;

        /// It scales up the AABB linked to the timber element.
        void scaleAABB(double scale);
        /** 
         * @brief It removes the duplicate tags from the timber object by checking center distances
         * 
         * @param distancethreshold the distance threshold between two tags' centers to consider two 
         * tags as duplicate.
         */
        void removeDuplicateTags(double distancethreshold = 0.5);

        /**
         * @brief Set the timber object's tags from the yaml file containing the planes coordinates
         * 
         * @param filename path of the yaml file
         */
        void setPlaneTagsFromYAML(const std::string& filename);
        /**
         * @brief Set the timber object's tags from the vector of TSRTag objects
         * 
         * @param planeTags vector of TSRTag objects
         */
        inline void setPlaneTags(std::vector<tslam::TSRTag> planeTags) {m_RTags = planeTags; };

        // /// It divides the tags attached to the timber in groups by radius search, and normal angle difference.
        // std::vector<std::vector<tslam::TSRTag>> divideTagsByRadiusAndNormal();

    public: __always_inline
        /// Get all the tags objects attached to the timber element.
        std::vector<tslam::TSRTag>& getPlaneTags() {return m_RTags; };
        /// Get the axis aligned box of the tags attached to the timber element.
        open3d::geometry::AxisAlignedBoundingBox& getAABB() {return m_AABB; };
        /// Get the point cloud of the tags' centers.
        open3d::geometry::PointCloud& getTagsCtrs() {return m_RtagsCtrs; };

    private:
        void compute() override;
        /// It computes the axis aligned bounding box of the tags and store it in a member variable.
        void computeAABB();
        /// It computes the point cloud of the tags' centers and store it in a member variable.
        void computeTagsCtrs();

    private:
        /// m_RTags the tag objects sticked to the timber piece.
        std::vector<tslam::TSRTag> m_RTags;
        /// m_RtagsCtrs the point cloud constituted by the tags' centers.
        open3d::geometry::PointCloud m_RtagsCtrs;
        /// m_Mesh the output mesh of the reconstructed object.
        open3d::geometry::TriangleMesh m_Mesh;
        ///  m_AABB the axis aligned bounding box of the tags.
        open3d::geometry::AxisAlignedBoundingBox m_AABB;
    };
}