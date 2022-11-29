#pragma once

#include <open3d/Open3D.h>
#include <Eigen/Core>

namespace tslam
{
    /**
     * @brief TSPlaneTag class responsible for storing the plane information
     * 
     */
    class TSPlaneTag
    {
    public:
        TSPlaneTag();
        ~TSPlaneTag() = default;
    
        /**
         * @brief Get the corners' coordinates of the planes
         * 
         * @return The coordinates per each point
         */
        inline std::vector<Eigen::Vector3f> getCorners() {return m_corners; };

        /**
         * @brief Set the corners' coordinates with a vector
         * 
         * @param corners vector of 4 corners
         */
        void setCorners(std::vector<Eigen::Vector3f> corners);
        void setCorners(Eigen::Vector3f A, Eigen::Vector3f B, Eigen::Vector3f C, Eigen::Vector3f D);

        inline Eigen::Vector3f getCornerA() {return m_corners[0]; };
        inline Eigen::Vector3f getCornerB() {return m_corners[1]; };
        inline Eigen::Vector3f getCornerC() {return m_corners[2]; };
        inline Eigen::Vector3f getCornerD() {return m_corners[3]; };

        /**
         * @brief Get the normal vector of the plane
         */
        inline void setID(uint id) {m_id = id; };
        /**
         * @brief Get the ID aruco marker of the plane
         * 
         * @return The id 
         */
        inline uint getID() {return m_id; };

        friend std::ostream& operator<<(std::ostream& os, TSPlaneTag& plane)
        {
            std::string id = std::to_string(plane.getID());
            os << "id: " << id << std::endl;
            return os;
        };

        /**
         * @brief Convert the plane to a open3d mesh
         * 
         * @return The newly created mesh
         */
        std::shared_ptr<open3d::geometry::TriangleMesh> toOpen3dMesh();

        /**
         * @brief Static method to fill a vector of TSPlaneTag from a yaml file containing their corners data
         * @param filename path to the map.yaml file
         * @param planes vector of TSPlaneTag objects
         */
        static void parseFromMAPYAML(const std::string& filename, std::vector<std::shared_ptr<TSPlaneTag>>& planes);

    private:
        std::vector<Eigen::Vector3f> m_corners;
        uint m_id;
        open3d::geometry::TriangleMesh m_PlaneMesh;
    };
}