#pragma once

#include "ts_rtag.hh"
#include "ts_rtstripe.hh"
#include "ts_geo_util.hh"
#include "ts_AABB.hh"

#include <Eigen/Core>

namespace tslam::Reconstruction
{
    /**
     * @brief TSTimber represents the timber element to be processed. It containes the
     * tags' geometrical and metdata info as well as the reconstruction.
     * 
     */
    class TSTimber : public TSCompute
    {
    public:
        TSTimber()
        {
            m_RTags = {};
        };
        TSTimber(std::vector<TSRTag> planeTags);
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
        inline void setPlaneTags(std::vector<TSRTag> planeTags)
        {
            m_RTags.clear();
            m_RTags = planeTags;
        };

        void setTSRTagsStripes(std::vector<std::shared_ptr<TSRTStripe>> tagsStripes)
        {
            m_TSRTagsStripes.clear();
            m_TSRTagsStripes = tagsStripes;

            // replace tags with the ones containes in the stripes
            m_RTags.clear();
            for (auto& stripe : m_TSRTagsStripes)
            {
                for (auto& tag : *stripe)
                {
                    m_RTags.push_back(tag);
                }
            }
        };
    
    public:  __attribute__((always_inline))  ///< clean out memory func
        /// Function to clean all the members and memory linked to the timber element
        void clean()
        {
            m_RTags.clear();
            m_TSRTagsStripes.clear();
            m_RtagsCtrs.clear();
        };
    
    public: __attribute__((always_inline))  ///< tag funcs
        /// Get all the tags objects attached to the timber element.
        std::vector<TSRTag> getPlaneTags() {return m_RTags; };
        /// Get number of tags after removing duplicates.
        int getNbrTags() {return m_RtagsCtrs.size(); };
        /// Get the tags organized in stripes.
        std::vector<std::shared_ptr<TSRTStripe>>& getTSRTagsStripes() {return m_TSRTagsStripes; };
        /// Get the two corners defining the axis aligned bounding box of the tags.
        Eigen::Vector3d& getAABBMin() {return m_AABB.getMin(); };
        Eigen::Vector3d& getAABBMax() {return m_AABB.getMax(); };
        /// Get the point cloud of the tags' centers.
        std::vector<Eigen::Vector3d>& getTagsCtrs() {return m_RtagsCtrs; };
        /// Get if the object has tags attached to it.
        bool hasTags() {return (m_RTags.size() > 0) ? true : false; };

    private:  ///< compute funcs
        void compute() override;
        /// It computes the axis aligned bounding box of the tags and store it in a member variable.
        void computeAABB();
        /// It computes the point cloud of the tags' centers and store it in a member variable.
        void computeTagsCtrs();

    private:  ///< member vars
        /// m_RTags the tag objects sticked to the timber piece.
        std::vector<TSRTag> m_RTags;
        /// m_TSRTagsStripes the tags organized in stripes.
        std::vector<std::shared_ptr<TSRTStripe>> m_TSRTagsStripes;
        /// m_RtagsCtrs the point cloud constituted by the tags' centers.
        std::vector<Eigen::Vector3d> m_RtagsCtrs;
        // ///  m_AABB the axis aligned bounding box of the tags.
        TSAABB m_AABB;
        
    };
}