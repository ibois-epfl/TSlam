#include "ts_timber.hh"


namespace tslam
{
    TSTimber::TSTimber() {m_planeTags = std::vector<std::shared_ptr<tslam::TSPlaneTag>>({}); };

    void TSTimber::setPlaneTagsFromYAML(const std::string& filename)
    {
        tslam::TSPlaneTag::parseFromMAPYAML(filename, m_planeTags);

    };
}