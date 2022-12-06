#pragma once

#include "ts_object.hh"

#include <open3d/Open3D.h>
#include <Eigen/Core>

namespace tslam
{
    /// TSTPlane struct to store the plane equation.
    struct TSTPlane
    {
        TSTPlane() {};
        TSTPlane(double a, double b, double c, double d)
            : a(a), b(b), c(c), d(d)
        {};
        ~TSTPlane() = default;

        double a, b, c, d;  // ax+by+cz=d
    };
    
    /// TSRTag class responsible for storing the plane information.
    class TSRTag : public TSObject
    {
    public:
        TSRTag() {};
        ~TSRTag() = default;

        /**
         * @brief Set the corners' coordinates of the planes
         * 
         * @param corners vector of 4 corners' coordinates
         */
        void setCorners(std::vector<Eigen::Vector3d> corners);
        /** @brief Set the corners' coordinates of the planes
         * 
         * @overload
        */
        void setCorners(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D);
        inline void setID(uint id) {m_Id = id; };

        inline std::vector<Eigen::Vector3d>& getCorners() {return m_Corners; }; 
        inline Eigen::Vector3d& getCornerA() {return m_Corners[0]; };
        inline Eigen::Vector3d& getCornerB() {return m_Corners[1]; };
        inline Eigen::Vector3d& getCornerC() {return m_Corners[2]; };
        inline Eigen::Vector3d& getCornerD() {return m_Corners[3]; };
        inline uint& getID() {return m_Id; };
        inline open3d::geometry::TriangleMesh& getOpen3dMesh() {return m_PlaneMesh; };
        inline Eigen::Vector3d& getCenter() {return m_Center; };
        inline TSTPlane& getPlane() {return m_Plane; };

        /**
         * @brief Static method to fill a vector of TSRTag from a yaml file containing their corners data
         * @param filename path to the map.yaml file
         * @param planes vector of TSRTag objects
         */
        static void parseFromMAPYAML(const std::string& filename, std::vector<TSRTag>& tags);
    
        friend std::ostream& operator<<(std::ostream& os, TSRTag& plane)
            {
                std::string id = std::to_string(plane.getID());
                os << "id: " << id << std::endl;
                return os;
            };

    private:
        /// Compute the intrinsic properties from the corners and it sets the obj members.
        void compute() override;
        void computeCenter();
        void computePlaneEquation();
        void computeOpen3dMesh();
    
    public: __always_inline
#ifdef TSLAM_REC_DEBUG
        void setColor(Eigen::Vector3d clr) {m_Color = clr; };
        Eigen::Vector3d& getColor() {return m_Color; };
#else
        void setColor(Eigen::Vector3d clr) {};
        Eigen::Vector3d& getColor() {};
#endif

    private:
        uint m_Id;
        std::vector<Eigen::Vector3d> m_Corners;
        TSTPlane m_Plane;
        open3d::geometry::TriangleMesh m_PlaneMesh;
        Eigen::Vector3d m_UnorientedPlaneNormal;
        Eigen::Vector3d m_Center;

#ifdef TSLAM_REC_DEBUG
        Eigen::Vector3d m_Color;
#endif
    };
}