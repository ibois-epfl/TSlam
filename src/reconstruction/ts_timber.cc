#include "ts_timber.hh"


namespace tslam
{
    TSTimber::TSTimber(std::vector<tslam::TSRTag> planeTags)
        : m_RTags(planeTags)
    {
        this->compute();
    }

    void TSTimber::scaleAABB(double scale)
    {
        this->m_AABB.Scale(scale, this->m_AABB.GetCenter());
    }
    
    void TSTimber::setPlaneTagsFromYAML(const std::string& filename)
    {
        tslam::TSRTag::parseFromMAPYAML(filename, m_RTags);

        this->compute();
    };

    void TSTimber::compute()
    {
        this->computeAABB();
        this->computeTagsCtrs();

        //WIP >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        std::vector<tslam::TSRTag> tags = this->m_Timber.getPlaneTags();
        open3d::geometry::KDTreeFlann kdtree;
        kdtree.SetGeometry(open3d::geometry::PointCloud(this->m_RtagsCtrs));


        std::vector<int> indices;
        std::vector<double> distances;
        // double radius = 3.0;
        int knn = 3;

        kdtree.SearchKNN(this->getPlaneTags()[10].getCenter(), knn, indices, distances);

        std::cout << "indices: " << indices.size() << std::endl;

        // set different colors for each cluster
        Eigen::Vector3d RED = Eigen::Vector3d(1.0, 0.0, 0.0);

        for (int i = 0; i < indices.size(); i++)
        {
            this->getPlaneTags()[indices[i]].setColor(RED);
        }
        this->getPlaneTags()[10].setColor(Eigen::Vector3d(0.0, 1.0, 0.0));
    }
    void TSTimber::computeAABB()
    {
        open3d::geometry::PointCloud pntCld;
        for (auto& p : this->getPlaneTags())
        {
            pntCld.points_.push_back(p.getCenter());
        }
        this->m_AABB = pntCld.GetAxisAlignedBoundingBox();
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