#include "ts_timber.hh"


namespace tslam
{
    TSTimber::TSTimber() {};

    void TSTimber::setPlaneTagsFromYAML(const std::string& filename)
    {
        tslam::TSPlaneTag::parseFromMAPYAML(filename, m_planeTags);
    };
}