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
        m_corners = corners;
    }

}