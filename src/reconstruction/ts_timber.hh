#pragma once

#include "ts_plane_tag.hh"
#include <open3d/Open3D.h>

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
        TSTimber();
        TSTimber(std::vector<std::shared_ptr<tslam::TSPlaneTag>> planeTags)
            : m_planeTags(planeTags)
        {};
        ~TSTimber() = default;

        /**
         * @brief Set the timber object's tags from the yaml file containing the planes coordinates
         * 
         * @param filename path of the yaml file
         */
        void setPlaneTagsFromYAML(const std::string& filename);
        inline void setPlaneTags(std::vector<std::shared_ptr<tslam::TSPlaneTag>> planeTags) {m_planeTags = planeTags; };
        
        inline std::vector<std::shared_ptr<tslam::TSPlaneTag>> getPlaneTags() {return m_planeTags; };

    private:
        /**
         * @brief m_planeTags the tag objects sticked to the timber piece.
         */
        std::vector<std::shared_ptr<tslam::TSPlaneTag>> m_planeTags;
        /**
         * @brief m_mesh the output mesh of the reconstructed object.
         */
        open3d::geometry::TriangleMesh m_mesh;

    };
}