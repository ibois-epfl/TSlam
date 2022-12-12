
#pragma once

#include "ts_compute.hh"

#include <Eigen/Core>
#include <tuple>
#include <vector>
#include <iostream>

#include <iterator>  // FIXME: test if needed
#include <algorithm>  // FIXME: test if needed
#include <math.h>  // FIXME: test if needed

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

    public: __always_inline
        /**
         * @brief Check if the point is on the plane
         * 
         * @param point the point to test
         * @return true if the point is on the plane
         * @return false if the point is not on the plane
         */
        bool isPointOnPlane(Eigen::Vector3d point)
        {
            return (this->Normal.dot(point - this->Center) < 1e-5);
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
    
    public: __always_inline
        /**
         * @brief It computes the angle between two segments in degrees on a given plane
         * 
         * @param otherü[out] the other segment
         * @param plane the plane on which the angle is computed
         * @return double the computed angle in degrees
         */
        double angleDegOnPlane(const TSSegment& other, const TSPlane& plane) const
        {
            Eigen::Vector3d v1 = this->getDirection();
            Eigen::Vector3d v2 = other.getDirection();
            double angle = acos(v1.dot(v2));
            Eigen::Vector3d cross = v1.cross(v2);
            if (cross.dot(plane.Normal) < 0)
                angle = -angle;
            return angle * 180.0 / M_PI;
        };

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
        /**
         * @brief Check if a point is on the polygon first by checking if it is on the plane,
         * then by checking if it is on one of the polygon's segments.
         * 
         * @param point the point to check
         * @return true if the point is on the polygon
         * @return false if the point is not on the polygon
         */
        bool isPointOnPolygon(Eigen::Vector3d point)
        {
            if (!m_LinkedPlane.isPointOnPlane(point))
                return false;

            for (auto s : m_Segments)
                if (s.isPointOnSegment(point))
                    return true;
            return false;
        };

    public: __always_inline
        /**
         * @brief It splits the polygon in two polygons given a segment with extremities on the polygon
         * 
         * @param segment the splitting segment
         * @return std::tuple<TSPolygon, TSPolygon> the two splitted polygons
         */
        std::tuple<TSPolygon, TSPolygon> splitPolygon(TSSegment segment)
        {
            // check if the segment extremities are on the polygon
            // TODO: in this scenario we need to build the splitting functions for segments not on the polygon
            if (!this->isPointOnPolygon(segment.Origin()) || !this->isPointOnPolygon(segment.EndPoint()))
                return std::make_tuple(TSPolygon(), TSPolygon());



            std::vector<Eigen::Vector3d> pointsA, pointsB;

            pointsA.push_back(segment.Origin());
            pointsB.push_back(segment.EndPoint());
            for (uint i = 0; i < m_Points.size(); i++)
            {
                TSSegment testSeg = TSSegment(segment.Origin(), m_Points[i]);
                double angle = segment.angleDegOnPlane(testSeg, m_LinkedPlane);

                if (angle < 0)
                    pointsA.push_back(m_Points[i]);
                else if (angle > 0)
                    pointsB.push_back(m_Points[i]);
                else
                {
                    if (segment.isPointOnSegment(m_Points[i]))
                    {
                        pointsA.push_back(m_Points[i]);
                        pointsB.push_back(m_Points[i]);
                    }
                }
            }
            pointsA.push_back(segment.EndPoint());
            pointsB.push_back(segment.Origin());

            TSPolygon polyA = TSPolygon(pointsA, m_LinkedPlane);
            TSPolygon polyB = TSPolygon(pointsB, m_LinkedPlane);

            polyA.reorderClockwisePoints();
            polyB.reorderClockwisePoints();

            return std::make_tuple(polyA, polyB);

            // TSPolygon::reorderClockwisePoints(pointsA, pointsA.size(), m_LinkedPlane);
            // TSPolygon::reorderClockwisePoints(pointsB, pointsB.size(), m_LinkedPlane);

            // return std::make_tuple(TSPolygon(pointsA, m_LinkedPlane), TSPolygon(pointsB, m_LinkedPlane));
        };

        // /**
        //  * @brief Reorder the points in a clockwise order
        //  * 
        //  * @param points the points to reorder
        //  * @param point_count the number of points
        //  * @param plane the plane on which the points lie
        //  */
        // void reorderClockwisePoints(std::vector<Eigen::Vector3d>& points,
        //                             unsigned point_count,
        //                             const TSPlane& plane)
        // {
        //     Eigen::Vector3d center = Eigen::Vector3d(0.f, 0.f, 0.f);

        //     // compute the center of the polygon
        //     for  (unsigned i = 0; i < point_count; ++i)
        //         center += points[i];
        //     center /= (float)point_count;

        //     // sort the points in a clockwise order also for the case of plane.A < 0.f
        //     // (the plane is not normalized)
        //     std::sort(points.begin(), points.end(),
        //               [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b)
        //               {
        //                   Eigen::Vector3d a_center = a - center;
        //                   Eigen::Vector3d b_center = b - center;
        //                   float angle_a = std::atan2(a_center(1) * plane.A - a_center(0) * plane.B,
        //                                              a_center(0) * plane.A + a_center(1) * plane.B);
        //                   float angle_b = std::atan2(b_center(1) * plane.A - b_center(0) * plane.B,
        //                                              b_center(0) * plane.A + b_center(1) * plane.B);
        //                   return angle_a < angle_b;
        //               });
        // }
        /// Reorder the polygon vertices in a clockwise order
        void reorderClockwisePoints()
        {
            // TSPolygon::reorderClockwisePoints(m_Points, m_Points.size(), m_LinkedPlane);

            Eigen::Vector3d center = Eigen::Vector3d(0.f, 0.f, 0.f);

            // compute the center of the polygon
            for  (unsigned i = 0; i < m_Points.size(); ++i)
                center += m_Points[i];
            center /= (float)m_Points.size();

            // sort the points in a clockwise order also for the case of plane.A < 0.f
            // (the plane is not normalized)
            std::sort(m_Points.begin(), m_Points.end(),
                      [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b)
                      {
                          Eigen::Vector3d a_center = a - center;
                          Eigen::Vector3d b_center = b - center;
                          float angle_a = std::atan2(a_center(1) * m_LinkedPlane.A - a_center(0) * m_LinkedPlane.B,
                                                     a_center(0) * m_LinkedPlane.A + a_center(1) * m_LinkedPlane.B);
                          float angle_b = std::atan2(b_center(1) * m_LinkedPlane.A - b_center(0) * m_LinkedPlane.B,
                                                     b_center(0) * m_LinkedPlane.A + b_center(1) * m_LinkedPlane.B);
                          return angle_a < angle_b;
                      });

            this->compute();
        }

    private:
        /// The polygon's vertices
        std::vector<Eigen::Vector3d> m_Points;
        /// The polygon's center
        Eigen::Vector3d m_Center;
        /// The plane on which the polygon lies
        tslam::TSPlane m_LinkedPlane;
        /// The polygon's segments
        std::vector<TSSegment> m_Segments;
    };
}