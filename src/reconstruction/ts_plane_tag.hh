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
        void setCorners(std::vector<Eigen::Vector3d> corners);
        void setCorners(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D);
        inline void setID(uint id) {m_id = id; };

        inline std::vector<Eigen::Vector3d>& getCorners() {return m_corners; }; 
        inline Eigen::Vector3d& getCornerA() {return m_corners[0]; };
        inline Eigen::Vector3d& getCornerB() {return m_corners[1]; };
        inline Eigen::Vector3d& getCornerC() {return m_corners[2]; };
        inline Eigen::Vector3d& getCornerD() {return m_corners[3]; };
        inline uint& getID() {return m_id; };
        open3d::geometry::TriangleMesh& getOpen3dMesh();
        Eigen::Vector3d& getUnorientedPlaneNormal();
        Eigen::Vector3d& getCenter();

        /**
         * @brief Static method to fill a vector of TSPlaneTag from a yaml file containing their corners data
         * @param filename path to the map.yaml file
         * @param planes vector of TSPlaneTag objects
         */
        static void parseFromMAPYAML(const std::string& filename, std::vector<TSPlaneTag>& planes);
    
        friend std::ostream& operator<<(std::ostream& os, TSPlaneTag& plane)
            {
                std::string id = std::to_string(plane.getID());
                os << "id: " << id << std::endl;
                return os;
            };

    private:
        /** @brief Convert the plane to a open3d mesh */
        std::shared_ptr<open3d::geometry::TriangleMesh> toOpen3dMesh();
        /**
         * @brief Compute the onoriented plane normal of the tag, there is no guarantee that the normal is 
         * pointing outwards or inwards correctly
         * 
         * @param A corner A of the tag
         * @param B corner B of the tag
         * @param C corner C of the tag
         * @return Eigen::Vector3d The unoriented plane normal
         */
        Eigen::Vector3d computeUnorientedPlaneNormal(const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& C);
        /**
         * @brief Compute the center of the tag's polygon
         * 
         * @param corners corners of the tag
         * @return Eigen::Vector3d The center of the plane
         */
        Eigen::Vector3d computeCenter(std::vector<Eigen::Vector3d> corners);

    private:
        std::vector<Eigen::Vector3d> m_corners;
        uint m_id;
        open3d::geometry::TriangleMesh m_PlaneMesh;
        Eigen::Vector3d m_unorientedPlaneNormal;
        Eigen::Vector3d m_center;
    };
}