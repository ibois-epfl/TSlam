#include "ts_plane.hh"

namespace tslam
{
    TSPlane::TSPlane() {};

    void TSPlane::setCorners(std::vector<Eigen::Vector3f> corners)
    {
        if (corners.size() != 4)
        {
            throw std::invalid_argument("[ERROR]: corners must be 4");
        }
        m_corners.clear();
        m_corners = corners;
    }

    void TSPlane::setCorners(Eigen::Vector3f A, Eigen::Vector3f B, Eigen::Vector3f C, Eigen::Vector3f D)
    {
        m_corners.clear();
        m_corners.push_back(A);
        m_corners.push_back(B);
        m_corners.push_back(C);
        m_corners.push_back(D);
    }

}