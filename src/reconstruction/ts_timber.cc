#include "ts_timber.hh"


namespace tslam::Reconstruction
{
    TSTimber::TSTimber(std::vector<TSRTag> planeTags)
        : m_RTags(planeTags)
    {
        this->compute();
    }

    void TSTimber::scaleAABB(double scale)
    {
        this->m_AABB.scale(scale);
    }

    void TSTimber::removeDuplicateTags(double distancethreshold)
    {
        std::vector<TSRTag> tagsNoDuplTemp;
        std::shared_ptr<open3d::geometry::PointCloud> pntCldTemp = std::make_shared<open3d::geometry::PointCloud>();
        for (auto& p : this->getPlaneTags())
        {
            pntCldTemp->points_.push_back(p.getCenter());
        }

        std::shared_ptr<open3d::geometry::PointCloud> pcdDown = pntCldTemp->VoxelDownSample(distancethreshold);

        for (auto& p : pcdDown->points_)
        {
            for (auto& q : this->getPlaneTags())
            {
                if (p.isApprox(q.getCenter()))
                {
                    tagsNoDuplTemp.push_back(q);
                }
            }
        }

        this->m_RTags.clear();
        this->m_RTags = tagsNoDuplTemp;
    }

    void TSTimber::setPlaneTagsFromYAML(const std::string& filename)
    {
        TSRTag::parseFromMAPYAML(filename, this->m_RTags);
        this->removeDuplicateTags();
        this->compute();
    };

    void TSTimber::compute()
    {
        this->computeAABB();
        this->computeTagsCtrs();

    }
    void TSTimber::computeAABB()
    {
        std::vector<Eigen::Vector3d> pts;
        for (auto& p : this->getPlaneTags())
            pts.push_back(p.getCenter());

        this->m_AABB = TSAABB(pts);
    }
    
    void TSTimber::computeTagsCtrs()
    {
        open3d::geometry::PointCloud pntCld;
        for (auto& p : this->getPlaneTags())
        {
            pntCld.points_.push_back(p.getCenter());
        }
        this->m_RtagsCtrs = pntCld;
    }
}