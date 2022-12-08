
#pragma once

#include "ts_object.hh"

#include <Eigen/Core>

namespace tslam
{
    /// A simple struct to store a plane
    struct TSTPlane
    {
        TSTPlane() {};
        TSTPlane(double a, double b, double c, double d)
            : A(a), B(b), C(c), D(d), Normal(Eigen::Vector3d(a, b, c)), Center(Eigen::Vector3d(-a*d, -b*d, -c*d))
        {};
        TSTPlane(Eigen::Vector3d normal, Eigen::Vector3d center)
            : A(normal(0)), B(normal(1)), C(normal(2)), D(normal.dot(center)), Normal(normal), Center(center)
        {};
        ~TSTPlane() = default;

    public:
        Eigen::Vector3d Normal;
        Eigen::Vector3d Center;
        double A, B, C, D;  ///< eq: ax+by+cz=d
    };

    /// Utility struct to store a polygon geometry for intersections
    struct TSPolygon
    {
        TSPolygon() = default;
        TSPolygon(std::vector<Eigen::Vector3d> points, TSTPlane linkedPlane)
            : m_Points(points), m_LinkedPlane(linkedPlane)
        {
            this->findCenter();
        };
        ~TSPolygon() = default;

    private: __always_inline
        void findCenter()
        {
            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (auto p : m_Points)
                center += p;
            center /= m_Points.size();
            m_Center = center;
        };
    
    public: __always_inline
        void addPoint(Eigen::Vector3d point) {m_Points.push_back(point); findCenter(); };
        void setPoints(std::vector<Eigen::Vector3d> points) {m_Points = points; findCenter(); };
        void setLinkedPlane(TSTPlane linkedPlane) {m_LinkedPlane = linkedPlane; };

    public:__always_inline
        std::vector<Eigen::Vector3d>& getPoints() {return m_Points; };
        uint getNumPoints() {return m_Points.size(); };
        Eigen::Vector3d& getPoint(uint i) {return m_Points[i]; };
        Eigen::Vector3d& getCenter() {return m_Center; };
        Eigen::Vector3d getNormal() {return m_LinkedPlane.Normal; };
        /**
         * @brief Get the Linked Plane object
         * 
         * @return tslam::TSTPlane& it returns the plane on which the polygon lies
         */
        tslam::TSTPlane& getLinkedPlane() {return m_LinkedPlane; };
    
    private:
        std::vector<Eigen::Vector3d> m_Points;
        Eigen::Vector3d m_Center;
        /// The plane on which the polygon lies
        tslam::TSTPlane m_LinkedPlane;
    };
}