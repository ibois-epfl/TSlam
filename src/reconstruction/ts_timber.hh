#pragma once

#include "ts_plane_tag.hh"

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

        inline void setPlaneTags(std::vector<std::shared_ptr<tslam::TSPlaneTag>> planeTags) {m_planeTags = planeTags; };
        void setPlaneTagsFromYAML(const std::string& filename);
        inline std::vector<std::shared_ptr<tslam::TSPlaneTag>> getPlaneTags() {return m_planeTags; };

        // create begin and end iterators for the plane tags, TODO: implement iterator

    private:
        std::vector<std::shared_ptr<tslam::TSPlaneTag>> m_planeTags;
    };
}