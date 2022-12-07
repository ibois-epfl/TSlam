#include "ts_timber.hh"
#include <math.h>  ///< for acos()

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
        // this->m_RtagsCtrs.EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(20));


        // // ======================================================
        // kdtree 
        open3d::geometry::KDTreeFlann kdtree;
        kdtree.SetGeometry(open3d::geometry::PointCloud(this->m_RtagsCtrs));


        std::vector<int> indices;
        std::vector<double> distances;
        int knn = 2;

        // for (int i = 0; i < this->getPlaneTags().size(); i++)
        // {
        //     // find k nearest neighbor
        //     kdtree.SearchKNN(this->getPlaneTags()[i].getCenter(), knn, indices, distances);
        //     // std::cout << "indices: " << indices[0] << " " << indices[1] << std::endl;

        //     // compute angle between normals
        //     Eigen::Vector3d normal1 = this->getPlaneTags()[i].getNormal();
        //     Eigen::Vector3d normal2 = this->getPlaneTags()[indices[1]].getNormal();

        //     // calculate angle between normals
        //     double angle = std::acos(normal1.dot(normal2) / (normal1.norm() * normal2.norm()));

        //     std::cout << "angle btw index " << i << " and " << indices[1] << ": " << angle << std::endl;
        
        // }

        // this->getPlaneTags()[246].setColor(Eigen::Vector3d(1.0, 0.0, 0.0));  // DEBUG
        // this->getPlaneTags()[157].setColor(Eigen::Vector3d(0.0, 1.0, 0.0));


        // TEST
        // color the tags based on the angle between normals
        // what we do here is detecting the creases
        for (int i = 0; i < this->getPlaneTags().size(); i++)
        {
            // find k nearest neighbor
            kdtree.SearchKNN(this->getPlaneTags()[i].getCenter(), knn, indices, distances);
            // std::cout << "indices: " << indices[0] << " " << indices[1] << std::endl;

            // compute angle between normals
            Eigen::Vector3d normal1 = this->getPlaneTags()[i].getNormal();
            Eigen::Vector3d normal2 = this->getPlaneTags()[indices[1]].getNormal();

            // calculate angle between normals
            double angle = std::acos(normal1.dot(normal2) / (normal1.norm() * normal2.norm()));
            // convert angles to degrees
            angle = angle * 180 / M_PI;

            std::cout << "angle btw index " << i << " and " << indices[1] << ": " << angle << std::endl;
        
            if (angle < 10)  ///<--- this is the threshold/sensibility of the solver
            {
                this->getPlaneTags()[i].setColor(Eigen::Vector3d(1.0, 0.0, 0.0));
            }
            else
            {
                this->getPlaneTags()[i].setColor(Eigen::Vector3d(0.0, 1.0, 0.0));
            }
        }




        // // set different colors for each cluster
        // Eigen::Vector3d RED = Eigen::Vector3d(1.0, 0.0, 0.0);
        // for (int i = 0; i < indices.size(); i++)
        // {
        //     this->getPlaneTags()[indices[i]].setColor(RED);
        // }
        // this->getPlaneTags()[10].setColor(Eigen::Vector3d(0.0, 1.0, 0.0));



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