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

    void TSTimber::removeDuplicateTags(double distancethreshold)
    {
        std::vector<tslam::TSRTag> tagsNoDuplTemp;
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
        tslam::TSRTag::parseFromMAPYAML(filename, m_RTags);
        this->removeDuplicateTags();
        this->compute();
    };

    void TSTimber::compute()
    {
        this->computeAABB();
        this->computeTagsCtrs();


        //WIP >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        // std::vector<tslam::TSRTag> tagsGroup;

        // evaluate normals of point cloud
        this->m_RtagsCtrs.EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(20));


        // // ======================================================
        // kdtree 
        open3d::geometry::KDTreeFlann kdtree;
        kdtree.SetGeometry(open3d::geometry::PointCloud(this->m_RtagsCtrs));


        std::vector<int> indices;
        std::vector<double> distances;
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