
#pragma once

#include "ts_compute.hh"

#include <Eigen/Core>
#include <tuple>
#include <vector>
#include <iostream>

#include <iterator>  // FIXME: test if needed
#include <algorithm>  // FIXME: test if needed


namespace tslam
{
    /// A simple struct to store a plane
    struct TSPlane
    {
        TSPlane() {};
        TSPlane(double a, double b, double c, double d)
            : A(a), B(b), C(c), D(d), Normal(Eigen::Vector3d(a, b, c)), Center(Eigen::Vector3d(-a*d, -b*d, -c*d))
        {};
        TSPlane(Eigen::Vector3d normal, Eigen::Vector3d center)
            : A(normal(0)), B(normal(1)), C(normal(2)), D(normal.dot(center)), Normal(normal), Center(center)
        {};
        ~TSPlane() = default;
    
    public: __always_inline
        bool operator!=(const TSPlane& other) const
        {
            return (this->Normal != other.Normal || this->Center != other.Center);
        };
        bool operator==(const TSPlane& other) const
        {
            return (this->Normal == other.Normal && this->Center == other.Center);
        };

    public:
        Eigen::Vector3d Normal;
        Eigen::Vector3d Center;
        double A, B, C, D;  ///< eq: ax+by+cz=d
    };

    /// A simple struct to store a segment object
    struct TSSegment
    {
        TSSegment() = default;
        TSSegment(Eigen::Vector3d p1, Eigen::Vector3d p2)
            : P1(p1), P2(p2)
        {};
        ~TSSegment() = default;
    
    public: __always_inline
        bool operator!=(const TSSegment& other) const
        {
            return (this->P1 != other.P1 || this->P2 != other.P2);
        };
        bool operator==(const TSSegment& other) const
        {
            return (this->P1 == other.P1 && this->P2 == other.P2);
        };

    public: __always_inline
        // FIXME: potential bug here for splitPolygon()?
        bool isPointOnSegment(Eigen::Vector3d point) const
        {
            Eigen::Vector3d v1 = point - this->P1;
            Eigen::Vector3d v2 = point - this->P2;
            Eigen::Vector3d v3 = this->P2 - this->P1;
            double cross = v1.cross(v2).norm();
            double dot = v1.dot(v2);
            double l = v3.norm();
            return (cross < 1e-5 && dot < 1e-5 && l > 1e-5);
        };
    
    public: __always_inline
        Eigen::Vector3d getDirection() const { return (this->P2 - this->P1).normalized(); };
        Eigen::Vector3d getCenter() const { return (this->P1 + this->P2) / 2.0; };
        Eigen::Vector3d& Origin() { return this->P1; };
        Eigen::Vector3d& EndPoint() { return this->P2; };

    public:
        Eigen::Vector3d P1, P2;
    };

    /// Utility struct to store a polygon geometry for intersections
    struct TSPolygon : public TSCompute
    {
        TSPolygon() = default;
        TSPolygon(std::vector<Eigen::Vector3d> points, TSPlane linkedPlane)
            : m_Points(points), m_LinkedPlane(linkedPlane)
        {
            this->compute();
        };
        ~TSPolygon() = default;

    public: __always_inline
        bool operator!=(const TSPolygon& other) const
        {
            // check if  all the points are the same
            if (this->m_Points.size() != other.m_Points.size())
                return true;
            for (uint i = 0; i < this->m_Points.size(); i++)
                if (this->m_Points[i] != other.m_Points[i])
                    return true;
            return false;
        };
        bool operator==(const TSPolygon& other) const
        {
            if (this->m_Points.size() != other.m_Points.size())
                return false;
            for (uint i = 0; i < this->m_Points.size(); i++)
                if (this->m_Points[i] != other.m_Points[i])
                    return false;
            return true;
        };
        uint size() {return m_Segments.size(); };
        TSSegment& operator[](uint i) {return m_Segments[i]; };

    private: __always_inline
        void compute() override
        {
            this->computeCenter();
            this->computeSegments();
        };
        void computeCenter()
        {
            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (auto p : m_Points)
                center += p;
            center /= m_Points.size();
            m_Center = center;
        };
        void computeSegments()
        {
            m_Segments.clear();

            for (uint i = 0; i < m_Points.size(); i++)
            {
                uint j = (i + 1) % m_Points.size();
                m_Segments.push_back(TSSegment(m_Points[i], m_Points[j]));
            }
        }
    
    public: __always_inline
        void addPoint(Eigen::Vector3d point) {m_Points.push_back(point); compute(); };
        void setPoints(std::vector<Eigen::Vector3d> points) {m_Points = points; compute(); };
        void setLinkedPlane(TSPlane linkedPlane) {m_LinkedPlane = linkedPlane; };

    public:__always_inline
        std::vector<Eigen::Vector3d>& getPoints() {return m_Points; };
        uint getNumPoints() {return m_Points.size(); };
        Eigen::Vector3d& getPoint(uint i) {return m_Points[i]; };
        Eigen::Vector3d& getCenter() {return m_Center; };
        Eigen::Vector3d getNormal() {return m_LinkedPlane.Normal; };
        /**
         * @brief Get the Linked Plane object
         * 
         * @return tslam::TSPlane& it returns the plane on which the polygon lies
         */
        tslam::TSPlane& getLinkedPlane() {return m_LinkedPlane; };
        /**
         * @brief Get the Segments object
         * 
         * @return std::vector<TSSegment>& it returns the polygon's segments
         */
        std::vector<TSSegment>& getSegments() {return m_Segments; };
    
    public: __always_inline
        std::tuple<bool, TSPolygon, TSPolygon> splitPolygon(TSSegment segment)
        {
            // check that the segments extremes w std::runtime_error("TSPolygon::splitPolygon: segment extremes not found on polygon");
            
            // FIXME: we need a check if segment's endpoitns are on the polygon
            // TODO: this function can be replaced by a TSPolygon function
            // bool areSplitSegEndsOnPolygon = false;
            // for (auto s : m_Segments)
            // {
            //     if (s.isPointOnSegment(segment.P1) && s.isPointOnSegment(segment.P2))
            //     {
            //         areSplitSegEndsOnPolygon = true;
            //         break;
            //     }
            // }
            // if (!areSplitSegEndsOnPolygon)
            //     return std::make_tuple(false, TSPolygon(), TSPolygon());

            // split the polygon into two polygons with clockwise ordering
            TSPolygon poly1, poly2;
            std::vector<Eigen::Vector3d> points1, points2;
            bool found = false;
            for (uint i = 0; i < m_Points.size(); i++)
            {
                uint j = (i + 1) % m_Points.size();
                if (segment == TSSegment(m_Points[i], m_Points[j]))
                {
                    found = true;
                    continue;
                }
                if (!found)
                    points1.push_back(m_Points[i]);
                else
                    points2.push_back(m_Points[i]);
            }
            points1.push_back(segment.P1);
            points1.push_back(segment.P2);
            points2.push_back(segment.P1);
            points2.push_back(segment.P2);

            // reorder the points
            if (TSSegment(points1[0], points1[1]).getDirection().dot(points1[2] - points1[0]) < 0)
                std::reverse(points1.begin(), points1.end());
            if (TSSegment(points2[0], points2[1]).getDirection().dot(points2[2] - points2[0]) < 0)
                std::reverse(points2.begin(), points2.end());



            // create a tuple of the two polygons
            return std::make_tuple(true, TSPolygon(points1, m_LinkedPlane), TSPolygon(points2, m_LinkedPlane));
        };

    private:
        std::vector<Eigen::Vector3d> m_Points;
        Eigen::Vector3d m_Center;
        /// The plane on which the polygon lies
        tslam::TSPlane m_LinkedPlane;
        /// The polygon's segments
        std::vector<TSSegment> m_Segments;
    };
}