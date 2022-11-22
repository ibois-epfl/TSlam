#pragma once

#include <open3d/Open3D.h>
#include <Eigen/Core>


namespace tslam
{
    class TSPlane
    {
    public:
        TSPlane();
        ~TSPlane() = default;
    
        inline std::vector<Eigen::Vector3f> getCorners() {return m_corners; };
        inline uint getID() {return m_id; };

        void setCorners(std::vector<Eigen::Vector3f> corners);
        void setID(uint id) {m_id = id; };

    private:
        std::vector<Eigen::Vector3f> m_corners;
        uint m_id;
    };
}