#pragma once

#include "ts_rtag.hh"
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
        TSTimber() {m_RTags = {}; };
        TSTimber(std::vector<tslam::TSRTag> planeTags)
            : m_RTags(planeTags)
        {};
        ~TSTimber() = default;

        /**
         * @brief Set the timber object's tags from the yaml file containing the planes coordinates
         * 
         * @param filename path of the yaml file
         */
        void setPlaneTagsFromYAML(const std::string& filename);
        inline void setPlaneTags(std::vector<tslam::TSRTag> planeTags) {m_RTags = planeTags; };
        
        inline std::vector<tslam::TSRTag> getPlaneTags() {return m_RTags; };

    private:
        /**
         * @brief m_RTags the tag objects sticked to the timber piece.
         */
        std::vector<tslam::TSRTag> m_RTags;
        /**
         * @brief m_Mesh the output mesh of the reconstructed object.
         */
        open3d::geometry::TriangleMesh m_Mesh;

    };
}