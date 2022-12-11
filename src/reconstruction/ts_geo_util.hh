
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
        std::tuple<bool, TSPolygon, TSPolygon> splitPolygon(TSSegment segment)
        {
            // output polygons
            TSPolygon poly1, poly2;

            // get the segment's extremes
            Eigen::Vector3d splitPtA = segment.Origin();
            Eigen::Vector3d splitPtB = segment.EndPoint();

            std::vector<Eigen::Vector3d> pointsA, pointsB;

            pointsA.push_back(splitPtA);
            pointsB.push_back(splitPtB);


            // get the vertices on the right and left of the segment
            for (uint i = 0; i < m_Points.size(); i++)
            {
                // // this is checking the case in which the segment is on top of a polygon vertex (?)
                // if (m_Points[i] == splitPtA || m_Points[i] == splitPtB)
                //     continue;

                TSSegment testSeg = TSSegment(splitPtA, m_Points[i]);
                double angle = segment.angleDegOnPlane(testSeg, m_LinkedPlane);
                // double angle = segment.angleDeg(testSeg);
                std::cout << "angle: " << angle << std::endl;

                if (angle < 0)
                    pointsA.push_back(m_Points[i]);
                else if (angle > 0)
                    pointsB.push_back(m_Points[i]);
                else
                {
                    // check if the point is on the segment
                    if (segment.isPointOnSegment(m_Points[i]))
                    {
                        pointsA.push_back(m_Points[i]);
                        pointsB.push_back(m_Points[i]);
                    }
                }

                // // if the angle is less than 90 degrees, the point is on the right
                // if (angle < 90.0)
                // {
                //     pointsA.push_back(m_Points[i]);
                // }
                // // if the angle is greater than 90 degrees, the point is on the left
                // else if (angle > 90.0)
                // {
                //     pointsB.push_back(m_Points[i]);
                // }
                // // if the angle is 90 degrees, the point is on the segment
                // else
                // {
                //     // check if the point is on the segment
                //     if (segment.isPointOnSegment(m_Points[i]))
                //     {
                //         pointsA.push_back(m_Points[i]);
                //         pointsB.push_back(m_Points[i]);
                //     }
                // }

                
            }

            pointsA.push_back(splitPtB);
            pointsB.push_back(splitPtA);

            // print the number of points in both polygons
            std::cout << "total segments: " << m_Segments.size() << std::endl;
            std::cout << "gt: " << (m_Segments.size() + 4) << std::endl;

            std::cout << "found: " << (pointsA.size() + pointsB.size()) << std::endl;

            std::cout << "ptsCount1: " << pointsA.size() << std::endl;
            std::cout << "ptsCount2: " << pointsB.size() << std::endl;
            std::cout << "============================================" << std::endl;


            // this->reorderClockwisePoints(pointsA, pointsA.size(), m_LinkedPlane);
            // this->reorderClockwisePoints(pointsB, pointsB.size(), m_LinkedPlane);


            // create a tuple of the two polygons
            return std::make_tuple(true, TSPolygon(pointsA, m_LinkedPlane), TSPolygon(pointsB, m_LinkedPlane));
        };

        // TODO: TEST
        void reorderClockwisePoints(std::vector<Eigen::Vector3d>& points,
                                    unsigned point_count,
                                    const TSPlane& plane)
        {
            Eigen::Vector3d center = Eigen::Vector3d(0.f, 0.f, 0.f);

            // compute the center of the polygon
            for  (unsigned i = 0; i < point_count; ++i)
                center += points[i];
            center /= (float)point_count;

            // sort the points in a clockwise order also for the case of plane.A < 0.f
            std::sort(points.begin(), points.end(), [plane, center](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
            {
                Eigen::Vector3d v1 = p1 - center;
                Eigen::Vector3d v2 = p2 - center;

                float dot = v1.dot(plane.Normal);
                float det = v1.x() * v2.y() - v1.y() * v2.x();

                return (det < 0.f) ? true : (det > 0.f) ? false : (dot < 0.f);
            });

            // if plane.A < 0.f, the points are ordered in a clockwise order, otherwise they are ordered in a counter-clockwise order
            if (plane.A < 0.f)
                std::reverse(points.begin(), points.end());
            

        }

    private:
        std::vector<Eigen::Vector3d> m_Points;
        Eigen::Vector3d m_Center;
        /// The plane on which the polygon lies
        tslam::TSPlane m_LinkedPlane;
        /// The polygon's segments
        std::vector<TSSegment> m_Segments;
    };
}