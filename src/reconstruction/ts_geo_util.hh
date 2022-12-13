
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
        // FIXME: to be tested!
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
        // TODO: set the tolerance as aparameter (1e-5)?
        /// it checks if a point is on a segment, also when the point is on the start or end of the segment
        bool isPointOnSegment(Eigen::Vector3d point) const
        {

            double AB = sqrt(pow(this->P2(0) - this->P1(0), 2) + pow(this->P2(1) - this->P1(1), 2) + pow(this->P2(2) - this->P1(2), 2));
            double AP = sqrt(pow(point(0) - this->P1(0), 2) + pow(point(1) - this->P1(1), 2) + pow(point(2) - this->P1(2), 2));
            double PB = sqrt(pow(this->P2(0) - point(0), 2) + pow(this->P2(1) - point(1), 2) + pow(this->P2(2) - point(2), 2));

            std::cout << "result" << abs(AB - (AP + PB)) << std::endl;

            return (abs(AB - (AP + PB)) < 1e-5);
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

        // FIXME: seems to work (suspicious)
        /**
         * @brief It intersects two segments and returns the intersection point
         * 
         * @param other[in] the other segment
         * @param intersection[out] the intersection point
         * @return true if the segments intersect
         * @return false if there is no intersection
         */
        bool intersect(const TSSegment& other, Eigen::Vector3d& intersection) const
        {
            // find the 3d intersection point of two segments
            Eigen::Vector3d A = P1;
            Eigen::Vector3d B = P2;
            Eigen::Vector3d C = other.P1;
            Eigen::Vector3d D = other.P2;

            Eigen::Vector3d AB = B - A;
            Eigen::Vector3d CD = D - C;

            Eigen::Vector3d cross = AB.cross(CD);
            if (cross.norm() == 0.f) return false;

            Eigen::Vector3d AC = C - A;

            double t = AC.cross(CD).dot(cross) / cross.squaredNorm();
            double u = AC.cross(AB).dot(cross) / cross.squaredNorm();

            // add a tolerance
            double tol = 1e-2;

            if (t >= 0.f - tol && t <= 1.f + tol && u >= 0.f - tol && u <= 1.f + tol)
            {
                Eigen::Vector3d ptTemp = A + t * AB;

                // FIXME: is point on segment is not working correctly
                if (this->isPointOnSegment(ptTemp) && other.isPointOnSegment(ptTemp))
                {
                    intersection = ptTemp;
                    return true;
                }
                else return false;
            }
            else return false;
        };
        // FIXME: to be tested, second version
        bool intersectV1(const TSSegment& other, Eigen::Vector3d& intersection) const
        {
            // find the 3d intersection point of two segments
            Eigen::Vector3d A = P1;
            Eigen::Vector3d B = P2;
            Eigen::Vector3d C = other.P1;
            Eigen::Vector3d D = other.P2;

            Eigen::Vector3d AB = B - A;
            Eigen::Vector3d CD = D - C;

            Eigen::Vector3d cross = AB.cross(CD);
            if (cross.norm() == 0.f) return false;

            Eigen::Vector3d AC = C - A;

            double t = AC.cross(CD).dot(cross) / cross.squaredNorm();
            double u = AC.cross(AB).dot(cross) / cross.squaredNorm();

            // add a tolerance
            double tol = 1e-2;

            if (t >= 0.f - tol && t <= 1.f + tol && u >= 0.f - tol && u <= 1.f + tol)
            {
                Eigen::Vector3d ptTemp = A + t * AB;

                // FIXME: is point on segment is not working correctly
                if (this->isPointOnSegment(ptTemp) && other.isPointOnSegment(ptTemp))
                {
                    intersection = ptTemp;
                    return true;
                }
                else return false;
            }
            else return false;
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
            // FIXME: not sure this is working or necessary
            // if (!m_LinkedPlane.isPointOnPlane(point))
            //     return false;

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
         * @param splitPoly[out] the two splitted polygons
         * @return true if the polygon has been splitted
         * @return false if the polygon has not been splitted or there is no intersection between the segment and the polygon
         */
        bool splitPolygon(TSSegment segment,
                          std::tuple<TSPolygon, TSPolygon>& splitPoly)
        {
            // // check if the segment extremities are not on the polygon
            // // TODO: in this scenario we need to build the splitting functions for segments not on the polygon
            // if (!this->isPointOnPolygon(segment.Origin()) || !this->isPointOnPolygon(segment.EndPoint()))
            //     // this is the scenario where the segment's end points are not on the polygon contour
            //     // we need to build the splitting functions for segments not on the polygon


            
            if (!this->isPointOnPolygon(segment.Origin()) || !this->isPointOnPolygon(segment.EndPoint()))
            {
                return false;  // TODO: debug, get rid of this when this block is ok

                // in this case one or both segment's end points are not on the polygon contour
                // we need to interesect the segment with the polygon contour
                // assuming that the polygon is convex, we must have two intersection points

                std::vector<Eigen::Vector3d> traversePoints;
                    Eigen::Vector3d ptTemp;
                    for (auto s : m_Segments)
                        if (s.intersectV1(segment, ptTemp)) traversePoints.push_back(ptTemp);
                
                if (this->isPointOnPolygon(segment.Origin()))
                {
                    std::cout << "[DEBUG] case A" << std::endl;
                    segment.P2 = traversePoints[1];
                }

                if (this->isPointOnPolygon(segment.EndPoint()))
                {
                    std::cout << "[DEBUG] case B" << std::endl;
                    segment.P1 = traversePoints[0];
                }

                if (!this->isPointOnPolygon(segment.Origin()) && !this->isPointOnPolygon(segment.EndPoint()))
                {
                    std::cout << "[DEBUG] case C" << std::endl;
                    if (traversePoints.size() != 2)
                    {
                        std::cout << "[ERROR] traversePoints.size(): " << traversePoints.size() << std::endl;
                        return false;
                    }
                    segment.P1 = traversePoints[0];
                    segment.P2 = traversePoints[1];
                }
            }


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

            std::get<0>(splitPoly) = polyA;
            std::get<1>(splitPoly) = polyB;

            return true;
        };

        /// Reorder the polygon vertices in a clockwise order
        void reorderClockwisePoints()
        {
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