/**
This file is part of TSLAM and distributed under the terms of the GPLv3.

Copyright (C) 2023 Andrea Settimi <andrea dot settimi at epfl dot ch>, Hong-Bin Yang <hong dash bin dot yang at epfl dot ch> (IBOIS, EPFL)

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Andrea Settimi and Hong-Bin Yang ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied Andrea Settimi and Hong-Bin Yang.
*/
#pragma once

#include "ts_compute.hh"
#include "ts_geo_util.hh"

#include <Eigen/Core>

namespace tslam::Reconstruction
{
    /// TSRTag class responsible for storing the plane information.
    class TSRTag : public TSCompute
    {
    public:
        TSRTag()
        {
            m_Id = -1;
            m_Corners = {};
            m_Center = Eigen::Vector3d::Zero();
            m_Plane = TSPlane();
            m_FaceIdx = -1;
        };
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
        inline void setFaceIdx(int idx) {m_FaceIdx = idx; };

    public: __attribute__((always_inline))
        std::vector<Eigen::Vector3d>& getCorners() {return m_Corners; }; 
        Eigen::Vector3d& getCornerA() {return m_Corners[0]; };
        Eigen::Vector3d& getCornerB() {return m_Corners[1]; };
        Eigen::Vector3d& getCornerC() {return m_Corners[2]; };
        Eigen::Vector3d& getCornerD() {return m_Corners[3]; };
        uint& getID() {return m_Id; };
        Eigen::Vector3d& getCenter() {return m_Center; };
        TSPlane& getPlane() {return m_Plane; };
        Eigen::Vector3d& getNormal() {m_Normal = m_Plane.Normal; return m_Normal; };
        Eigen::Vector3d& getAxisY() {m_AxisY = m_Plane.AxisY; return m_AxisY; };
        Eigen::Vector3d& getAxisX() {m_AxisX = m_Plane.AxisX; return m_AxisX; };
        uint& getFaceIdx() {return m_FaceIdx; };
    
    private:
        /// Compute the intrinsic properties from the corners and it sets the obj members.
        void compute() override;
        void computeCenter();
        void computePlaneEquation();
        void computeOpen3dMesh();
    
    public: __attribute__((always_inline))
        void setColor(Eigen::Vector3d clr) {m_Color = clr; };
        Eigen::Vector3d& getColor() {return m_Color; };

    public: __attribute__((always_inline))
        /**
         * @brief Compute the distance between two tags
         * 
         * @param other tag
         * @return double distance
         */
        double distance2Tag(TSRTag other)
        {
            // That is, given P1 = (x1,y1,z1) and P2 = (x2,y2,z2), the distance between P1 and P2 is given by d(P1,P2) = (x2 x1)2 + (y2 y1)2 + (z2 z1)2.
            Eigen::Vector3d& p1 = this->m_Center;
            Eigen::Vector3d& p2 = other.m_Center;

            double dist = (p2.x() - p1.x()) * (p2.x() - p1.x())
                          + (p2.y() - p1.y()) * (p2.y() - p1.y())
                          + (p2.z() - p1.z()) * (p2.z() - p1.z());
            return dist;
        };
        /**
         * @brief Compute the distance between a point and the tag's center
         * 
         * @see distance(TSRTag other)
         * @param p point
         * @return double distance
         */
        double distance2Pt(Eigen::Vector3d p)
        {
            Eigen::Vector3d& p1 = this->m_Center;
            double dist = (p.x() - p1.x()) * (p.x() - p1.x())
                          + (p.y() - p1.y()) * (p.y() - p1.y())
                          + (p.z() - p1.z()) * (p.z() - p1.z());
            return dist;
        };

    private:
        uint m_Id;
        std::vector<Eigen::Vector3d> m_Corners;
        TSPlane m_Plane;
        /// Normal vector of the plane linked to the tag
        Eigen::Vector3d m_Normal;
        /// Axis X,Y of the plane linked to the tag
        Eigen::Vector3d m_AxisX, m_AxisY;
        Eigen::Vector3d m_Center;
        /// Index to identify the face on which the tag is stick to
        uint m_FaceIdx;
        /// Color of the tag for visualizer
        Eigen::Vector3d m_Color;
    };
}