
#pragma once

#include "ts_compute.hh"

#include <Eigen/Core>
#include <tuple>
#include <vector>
#include <iostream>

#include <iterator>
#include <algorithm> 
#include <math.h>

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
            if (this->Normal.isApprox(other.Normal, 1e-5) && this->Center.isApprox(other.Center, 1e-5))
                return false;
            return true;
        };
        bool operator==(const TSPlane& other) const
        {
            if (!this->Normal.isApprox(other.Normal, 1e-5) && !this->Center.isApprox(other.Center, 1e-5))
                return false;
            return true;
        };
        friend std::ostream& operator<<(std::ostream& os, const TSPlane& plane)
        {
            os << "Plane: " << plane.Normal.transpose() << " - " << plane.Center.transpose();
            return os;
        };
    
    public: __always_inline
        // TODO: test
        /**
         * @brief It computes the distance between a point and the plane
         * 
         * @param point the point to compute the distance
         * @return double the distance
         */
        double distance(Eigen::Vector3d point)
        {
            return this->Normal.dot(point - this->Center);
        };

        // TODO: test me
        /**
         * @brief Project a point on the plane.
         * 
         * @param point[in] the point to project
         * @return Eigen::Vector3d the projected point
         */
        Eigen::Vector3d projectPoint(Eigen::Vector3d& point)
        {
            Eigen::Vector3d pt =  point - this->distance(point) * this->Normal;
            return pt;
        };
        // /**
        //  * @brief 
        //  * 
        //  * @param point[in] 
        //  * @param distThresh the distance threshold 
        //  * @return Eigen::Vector3d 
        //  * @see projectPoint(Eigen::Vector3d& point)
        //  */
        // Eigen::Vector3d projectPoint(Eigen::Vector3d& point, double distThresh)

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
            if (this->P1.isApprox(other.P1) && this->P2.isApprox(other.P2)
                        || this->P1.isApprox(other.P2) && this->P2.isApprox(other.P1))
                    return false;
            return true;
        };
        bool operator==(const TSSegment& other) const
        {
            return !(*this != other);
        };
        friend std::ostream& operator<<(std::ostream& os, const TSSegment& seg)
        {
            os << "Segment: " << seg.P1.transpose() << " - " << seg.P2.transpose();
            return os;
        };

    public: __always_inline
        // TODO: set the tolerance as aparameter (1e-5)?
        /// it checks if a point is on a segment, also when the point is on the start or end of the segment
        bool isPointOnSegment(Eigen::Vector3d point) const
        {
            double AB = sqrt(pow(this->P2(0) - this->P1(0), 2) + pow(this->P2(1) - this->P1(1), 2) + pow(this->P2(2) - this->P1(2), 2));
            double AP = sqrt(pow(point(0) - this->P1(0), 2) + pow(point(1) - this->P1(1), 2) + pow(point(2) - this->P1(2), 2));
            double PB = sqrt(pow(this->P2(0) - point(0), 2) + pow(this->P2(1) - point(1), 2) + pow(this->P2(2) - point(2), 2));

            return (abs(AB - (AP + PB)) < 1e-5);
        };

        bool isConnected(TSSegment other) const
        {
            return (this->P1.isApprox(other.P1, 1e-5) || this->P1.isApprox(other.P2, 1e-5) ||
                    this->P2.isApprox(other.P1, 1e-5) || this->P2.isApprox(other.P2, 1e-5));
        };
        /// It checks if there is intersection between two segments
        bool isIntersecting(TSSegment other)
        {
            Eigen::Vector3d tempIntersectPt;
            return this->intersect(other, tempIntersectPt);
        }

    public: __always_inline
        Eigen::Vector3d getDirection() const { return (this->P2 - this->P1).normalized(); };
        Eigen::Vector3d getCenter() const { return (this->P1 + this->P2) / 2.0; };
        Eigen::Vector3d& Origin() { return this->P1; };  //TODO: clean out this method
        Eigen::Vector3d& EndPoint() { return this->P2; };  // TODO: clean out this method
        Eigen::Vector3d getMidPoint()
        {
            Eigen::Vector3d midPoint = Eigen::Vector3d::Zero();
            midPoint = (this->P1 + this->P2) / 2.0;
            return midPoint;
        };

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
            // check that the segments are not the same
            if (*this == other) return false;

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

            double tol = 1e-10;

            if (t >= 0.f - tol && t <= 1.f + tol && u >= 0.f - tol && u <= 1.f + tol)
            {
                Eigen::Vector3d ptTemp = A + t * AB;

                if (this->isPointOnSegment(ptTemp) && other.isPointOnSegment(ptTemp))
                {
                    intersection = ptTemp;
                    return true;
                }
                else return false;
            }
            else return false;
        };

        // TODO: to be tested
        /** 
         * @brief It extends the segment to a given length
         * 
         * @param length the length to extend the segment
         */
        void extend(double length)
        {
            Eigen::Vector3d dir = this->getDirection();
            this->P2 = this->P1 + length * dir;
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
        bool operator==(const TSPolygon& other) const
        {
            ///< (a) check for number of vertices
            if (this->m_Points.size() == other.m_Points.size())
            {
                ///< (b) check for center (if the center is the same, the polygon is the same
                if (this->m_Center.isApprox(other.m_Center, 1e-5))
                    return true;
                
                ///< (c) check for vertices' coordinates
                uint NFoundPts = 0;
                for (uint i = 0; i < this->m_Points.size(); i++)
                {
                    for (uint j = 0; j < other.m_Points.size(); j++)
                    {
                        if (this->m_Points[i].isApprox(other.m_Points[j], 1e-5))
                            NFoundPts++;
                    }
                }
                if (NFoundPts == other.m_Points.size())
                    return true;
            }

            return false;
        };
        bool operator!=(const TSPolygon& other) const
        {
            return !(*this == other);
        };
        friend std::ostream& operator<<(std::ostream& os, const TSPolygon& polygon)
        {
            os << "Polygon center: " << polygon.m_Center << std::endl;
            os << "Polygon segments: " << std::endl;
            for (auto s : polygon.m_Segments)
                os << s << std::endl;
            os << "Polygon linked plane: " << polygon.m_LinkedPlane << std::endl;
            return os;
        };
        uint size() {return m_Segments.size(); };
        TSSegment& operator[](uint i) {return m_Segments[i]; };

    private: __always_inline
        void compute() override
        {
            this->computeVertices();
            this->computeCenter();
            this->computeSegments();
            this->computeArea();
        };
        void computeVertices()
        {
            if (m_Points.size() != 0)
                return;
            else if (m_Points.size() != 0 && m_Segments.size() != 0)
            {
                for (auto s : m_Segments)
                {
                    bool found = false;
                    for (auto p : m_Points)
                        if (p == s.P1)
                        {
                            found = true;
                            break;
                        }
                    if (!found)
                        m_Points.push_back(s.P1);
                }
            }
        };
        void computeCenter()
        {
            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (auto p : m_Points)
                center += p;
            center /= m_Points.size();
            m_Center = center;
        };
        // FIXME: problems with vertices storage
        void computeSegments()
        {
            m_Segments.clear();

            for (uint i = 0; i < m_Points.size(); i++)
            {
                uint j = (i + 1) % m_Points.size();
                m_Segments.push_back(TSSegment(m_Points[i], m_Points[j]));
            }
        }
        /// Compute the area of the polygon
        void computeArea()
        {
            double area = 0;
            for (uint i = 0; i < m_Points.size(); i++)
            {
                uint j = (i + 1) % m_Points.size();
                area += m_Points[i].x() * m_Points[j].y() - m_Points[j].x() * m_Points[i].y();
            }
            this->m_Area = area;
        };

    public: __always_inline
        void addPoint(Eigen::Vector3d point) {m_Points.push_back(point); compute(); };
        void removePoint(uint i) {m_Points.erase(m_Points.begin() + i); compute(); };
        void addSegment(TSSegment segment) {m_Segments.push_back(segment); };  // FIXME: problems with vertices storage
        void setPoints(std::vector<Eigen::Vector3d> points) {m_Points = points; compute(); };
        void setLinkedPlane(TSPlane linkedPlane) {m_LinkedPlane = linkedPlane; };

    public:__always_inline
        std::vector<Eigen::Vector3d>& getPoints() {return m_Points; };
        // TODO: refactor "point" term in "vertex" for polygon
        uint getNumPoints() {return m_Points.size(); };
        Eigen::Vector3d& getPoint(uint i) {return m_Points[i]; };
        int getVertexIndex(Eigen::Vector3d point)
        {
            for (uint i = 0; i < m_Points.size(); i++)
                if (m_Points[i].isApprox(point))
                    return i;
            return -1;
        };
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
        double getArea() {return m_Area; };

    public: __always_inline
        /**
         * @brief Check if a point is on the polygon by checking if it's on one of the polygon's segments.
         * 
         * @param point the point to check
         * @return true if the point is on the polygon
         * @return false if the point is not on the polygon
         */
        bool isPointOnPolygon(Eigen::Vector3d point)
        {
            for (auto s : m_Segments)
                if (s.isPointOnSegment(point))
                    return true;
            return false;
        };
        /** 
         * @brief check if the polygon is healthy
         * 
         * @return true if the polygon is valid
         * @return false if the polygon is not valid
         */
        bool isValid()
        {
            if (m_Points.size() < 3)
                return false;
            
            for (uint i = 0; i < m_Points.size(); i++)
            {
                uint j = (i + 1) % m_Points.size();
                if (!m_Segments[i].isConnected(m_Segments[j]))
                    return false;
            }

            if (this->getArea() < 1e-5)
                return false;

            return true;
        };
        // TODO: to be tested - not used
        /**
         * @brief Check if a point is inside the polygon.
         * 
         * @param point the point to check
         * @return true if the point is inside the polygon
         * @return false if the point is not inside the polygon
         */
        bool isPointInsidePolygon(Eigen::Vector3d point)
        {
            // if (!this->isPointOnPolygon(point))
            //     return false;

            Eigen::Vector3d point2 = point + 10 * this->getNormal();

            TSSegment seg = this->getSegments()[0];
            Eigen::Vector3d midPt = seg.getMidPoint();

            TSSegment segment(point, midPt);

            // extend the segment longer thant the mid point
            segment.extend(1000);

            std::vector<Eigen::Vector3d> intersections;
            Eigen::Vector3d tempIntersection;

            for (auto s : m_Segments)
            {
                if (s.intersect(segment, tempIntersection))
                {
                    intersections.push_back(tempIntersection);
                }
            }

            // // FIXME: check which checker is working
            // if (intersections.size() != 1)
            //     return false;
            // else
            //     return true;

            if (intersections.size() % 2 == 0)
                return false;
            else
                return true;
        };

    public: __always_inline
        /**
         * @brief It splits the polygon in two polygons given a segment.
         * ** NOTE **: the polygon must be convex and splitting is not working for segment's ends out of the polygon's contour.**
         * 
         * @param segment the splitting segment
         * @param splitPoly[out] the two splitted polygons
         * @return true if the polygon has been splitted
         * @return false if the polygon has not been splitted or there is no intersection between the segment and the polygon
         */
        bool splitPolygon(TSSegment segment, std::tuple<TSPolygon, TSPolygon>& splitPoly)
        {
            ///< (1) catch corner's cases and adjust the segment's end points for the splitting

            ///< (1.a) catch the case where the segment's end points are on the polygon's vertices
            for (auto& v : this->m_Points)
            {
                if (v.isApprox(segment.P1) || v.isApprox(segment.P2))
                    return false;
            }

            ///< (1.b) catch the case where both of the segment's end points are not on the polygon's contour
            if (!this->isPointOnPolygon(segment.Origin()) && !this->isPointOnPolygon(segment.EndPoint()))
            {
                Eigen::Vector3d interPt;
                std::vector<Eigen::Vector3d> interPts;
                for (auto& seg : this->m_Segments)
                {
                    if (seg.intersect(segment, interPt))
                        interPts.push_back(interPt);
                }
                if (interPts.size() == 2)
                {
                    segment.P1 = interPts[0];
                    segment.P2 = interPts[1];
                }
                else
                    return false;
            }
            ///< (1.c) catch the case where one of the segment's end points is not on the polygon's contour
            else if (!this->isPointOnPolygon(segment.Origin()) || !this->isPointOnPolygon(segment.EndPoint()))
            {
                bool isIntersect;
                Eigen::Vector3d interPt;

                for (auto& seg : this->m_Segments)
                {
                    isIntersect = seg.intersect(segment, interPt);
                    if (isIntersect && !interPt.isApprox(segment.P1) && !interPt.isApprox(segment.P2) && this->isPointOnPolygon(interPt))
                    {
                        if (!this->isPointOnPolygon(segment.P1))
                            segment.P1 = interPt;
                        else if (!this->isPointOnPolygon(segment.P2))
                            segment.P2 = interPt;
                    }
                }
                if (!this->isPointOnPolygon(segment.P1) || !this->isPointOnPolygon(segment.P2))
                    return false;
            }

            ///< (2) execute the splitting
            std::vector<Eigen::Vector3d> pointsA, pointsB;

            pointsA.push_back(segment.P1);
            pointsB.push_back(segment.P2);

            for (uint i = 0; i < m_Points.size(); i++)
            {
                TSSegment testSeg = TSSegment(segment.P1, m_Points[i]);
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

            pointsA.push_back(segment.P2);
            pointsB.push_back(segment.P1);

            TSPolygon polyA = TSPolygon(pointsA, m_LinkedPlane);
            TSPolygon polyB = TSPolygon(pointsB, m_LinkedPlane);

            polyA.reorderClockwisePoints();
            polyB.reorderClockwisePoints();


            ///< (3) test the polygon's sanity
            if (!(polyA.isValid() && polyB.isValid()))
                return false;

            std::get<0>(splitPoly) = polyA;
            std::get<1>(splitPoly) = polyB;

            return true;
        };
        /**
         * @brief It intersect the polygon with another one
         * 
         * @param poly[in] the polygon to intersect with
         * @param intersectedPolygons[out] the intersected polygons
         */
        bool intersectPolygon(TSPolygon& polyB, TSSegment& intersectedSegment)
        {
            std::vector<Eigen::Vector3d> tempIntersectPts;
            bool isIntersect = false;

            Eigen::Vector3d* intersectPt = new Eigen::Vector3d();

            for (int i = 0; i < this->size(); i++)
            {
                for (int j = 0; j < polyB.size(); j++)
                {
                    isIntersect = this->getSegments()[i].intersect(polyB[j], *intersectPt);

                    if (isIntersect)
                    {
                        tempIntersectPts.push_back(*intersectPt);
                    }
                }
            }
            delete intersectPt;

            if (tempIntersectPts.size() == 2)
            {
                intersectedSegment.P1 = tempIntersectPts[0];
                intersectedSegment.P2 = tempIntersectPts[1];
            }
            else return false;
            
            return true;
        }
        /// Reorder the polygon vertices in a clockwise order based on grahm scan algorithm
        void reorderClockwisePoints()
        {
            Eigen::Vector3d center = Eigen::Vector3d(0.f, 0.f, 0.f);

            // compute the center of the polygon
            for  (unsigned i = 0; i < m_Points.size(); ++i)
                center += m_Points[i];
            center /= (float)m_Points.size();

            // sort the points in a clockwise order also for the case of plane.A < 0.f
            // (the plane is not normalized)
            std::sort(m_Points.begin(),m_Points.end(),
                      [&](const Eigen::Vector3d& a, 
                      const Eigen::Vector3d& b)
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
        };
        /// Compare two polygons centers and return true if they are equal
        bool areCentersEqual(TSPolygon& other, float eps = 1e-3)
        {
            return this->m_Center.isApprox(other.m_Center, eps);
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
        /// the polygon's area
        double m_Area;
    };
}