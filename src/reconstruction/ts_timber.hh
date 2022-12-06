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
    class TSTimber
    {
    public:
        TSTimber() {m_RTags = {}; };
        TSTimber(std::vector<tslam::TSRTag> planeTags);
        ~TSTimber() = default;

        /** @brief It scales up the AABB linked to the timber element */
        void scaleAABB(double scale);

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
        
        /** @brief Get all the tags objects attached to the timber element */
        inline std::vector<tslam::TSRTag>& getPlaneTags() {return m_RTags; };
        /** @brief Get the axis aligned box of the tags attached to the timber element */
        inline open3d::geometry::AxisAlignedBoundingBox& getAABB() {return m_AABB; };
    
    private:
        void computeAABB();

    private:
        /// m_RTags the tag objects sticked to the timber piece.
        std::vector<tslam::TSRTag> m_RTags;
        /// @brief m_Mesh the output mesh of the reconstructed object.
        open3d::geometry::TriangleMesh m_Mesh;
        /// @brief m_AABB the axis aligned bounding box of the tags.
        open3d::geometry::AxisAlignedBoundingBox m_AABB;
    };
}