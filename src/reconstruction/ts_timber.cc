#include "ts_timber.hh"


namespace tslam
{
    void TSTimber::setPlaneTagsFromYAML(const std::string& filename)
    {
        tslam::TSRTag::parseFromMAPYAML(filename, m_RTags);
    };
}