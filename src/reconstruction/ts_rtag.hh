#pragma once

#include "ts_compute.hh"
#include "ts_geo_util.hh"

#include <open3d/Open3D.h>
#include <Eigen/Core>

namespace tslam
{
    enum TSRTagType
    {
        Unknown = 0,
        Edge = 1,
        Side = 2
    };
    
    /// TSRTag class responsible for storing the plane information.
    class TSRTag : public TSCompute
    {
    public:
        TSRTag() {};
        ~TSRTag() = default;

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

    public:
        inline bool isEdge() {return (m_Type==TSRTagType::Edge) ? true : false; };
        inline bool isSide() {return (m_Type==TSRTagType::Side) ? true : false; };

    public:
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
        inline void setType(TSRTagType type) {m_Type = type; };
        inline void setFaceIdx(int idx) {m_FaceIdx = idx; };

    public: __always_inline
        std::vector<Eigen::Vector3d>& getCorners() {return m_Corners; }; 
        Eigen::Vector3d& getCornerA() {return m_Corners[0]; };
        Eigen::Vector3d& getCornerB() {return m_Corners[1]; };
        Eigen::Vector3d& getCornerC() {return m_Corners[2]; };
        Eigen::Vector3d& getCornerD() {return m_Corners[3]; };
        uint& getID() {return m_Id; };
        open3d::geometry::TriangleMesh& getOpen3dMesh() {return m_PlaneMesh; };
        Eigen::Vector3d& getCenter() {return m_Center; };
        TSPlane& getPlane() {return m_Plane; };
        Eigen::Vector3d& getNormal() {m_Normal = m_Plane.Normal; return m_Normal; };
        TSRTagType& getType() {return m_Type; };
        uint& getFaceIdx() {return m_FaceIdx; };
        
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
        TSPlane m_Plane;
        open3d::geometry::TriangleMesh m_PlaneMesh;
        Eigen::Vector3d m_Normal;  ///< Normal vector of the plane NOT oriented (in(outwards))
        Eigen::Vector3d m_Center;
        TSRTagType m_Type;
        /// Index to identify the face on which the tag is stick to
        uint m_FaceIdx;

#ifdef TSLAM_REC_DEBUG
        Eigen::Vector3d m_Color;
#endif
    };
}