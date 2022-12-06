#include "ts_timber.hh"


namespace tslam
{
    TSTimber::TSTimber(std::vector<tslam::TSRTag> planeTags)
        : m_RTags(planeTags)
    {
        this->computeAABB();
    }

    void TSTimber::scaleAABB(double scale)
    {
        this->m_AABB.Scale(scale, this->m_AABB.GetCenter());
    }
    
    void TSTimber::setPlaneTagsFromYAML(const std::string& filename)
    {
        tslam::TSRTag::parseFromMAPYAML(filename, m_RTags);

        this->computeAABB();
    };

    void TSTimber::computeAABB()
    {
        open3d::geometry::PointCloud pntCld;
        for (auto& p : this->getPlaneTags())
        {
            pntCld.points_.push_back(p.getCenter());
        }
        this->m_AABB = pntCld.GetAxisAlignedBoundingBox();
    }
}