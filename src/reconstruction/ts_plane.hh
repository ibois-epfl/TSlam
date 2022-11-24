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

        void setCorners(std::vector<Eigen::Vector3f> corners);
        void setCorners(Eigen::Vector3f A, Eigen::Vector3f B, Eigen::Vector3f C, Eigen::Vector3f D);
        inline Eigen::Vector3f getCornerA() {return m_corners[0]; };
        inline Eigen::Vector3f getCornerB() {return m_corners[1]; };
        inline Eigen::Vector3f getCornerC() {return m_corners[2]; };
        inline Eigen::Vector3f getCornerD() {return m_corners[3]; };

        inline void setID(uint id) {m_id = id; };
        inline uint getID() {return m_id; };
        friend std::ostream& operator<<(std::ostream& os, TSPlane& plane)
        {
            std::string id = std::to_string(plane.getID());
            os << "id: " << id << std::endl;
            return os;
        };

        static void parseFromMAPYAML(const std::string& filename, std::vector<std::shared_ptr<tslam::TSPlane>>& planes);

    private:
        std::vector<Eigen::Vector3f> m_corners;
        uint m_id;
    };
}