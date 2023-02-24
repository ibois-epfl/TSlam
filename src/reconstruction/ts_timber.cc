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
        std::vector<Eigen::Vector3d> ctrs;

        for (auto& p : this->getPlaneTags())
            ctrs.push_back(p.getCenter());

        for (size_t i = 0; i < ctrs.size(); i++)
        {
            for (size_t j = i + 1; j < ctrs.size(); j++)
            {
                if (ctrs[i].isApprox(ctrs[j]))
                {
                    ctrs.erase(ctrs.begin() + j);
                }
            }
        }

        for (auto& p : ctrs)
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
        this->m_RtagsCtrs.clear();
        for (auto& p : this->getPlaneTags())
        {
            this->m_RtagsCtrs.push_back(p.getCenter());
        }
    }
}