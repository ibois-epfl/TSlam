#include "ts_timber.hh"


namespace tslam
{
    void TSTimber::setPlaneTagsFromYAML(const std::string& filename)
    {
        tslam::TSPlaneTag::parseFromMAPYAML(filename, m_planeTags);
    };
}