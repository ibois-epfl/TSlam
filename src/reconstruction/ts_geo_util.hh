
#pragma once

#include "ts_compute.hh"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tuple>
#include <vector>
#include <iostream>

#include <iterator>
#include <algorithm> 
#include <math.h>

namespace tslam::Reconstruction
{
    /// Struct interface for vector utility function
    struct TSVector
    {
    public: __always_inline  ///< utility static funcs
        /**
         * @brief Get the angle between two vectors
         * 
         * @param v1 the first vector
         * @param v2 the second vector
         * @return double the angle between the two vectors
         */
        static double angleBetweenVectors(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
        {
            double angle = std::acos(v1.dot(v2) / (v1.norm() * v2.norm()));
            if (std::isnan(angle))
                return 0.0;
            return (angle * 180 / M_PI);
        }
        /**
         * @brief Get the distance between two points
         * 
         * @param a the first point
         * @param b the second point
         * @return double the distance between the two points
         */
        static double distance(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
        {
            return std::sqrt(std::pow(a(0) - b(0), 2) + std::pow(a(1) - b(1), 2) + std::pow(a(2) - b(2), 2));
        }

    public: __always_inline  ///< custom WorldAxis
        /**
         * @brief Get the X axis
         * 
         * @return Eigen::Vector3d the X axis
         */
        static Eigen::Vector3d AxisX() { return Eigen::Vector3d(1, 0, 0); }
        /**
         * @brief Get the Y axis
         * 
         * @return Eigen::Vector3d the Y axis
         */
        static Eigen::Vector3d AxisY() { return Eigen::Vector3d(0, 1, 0); }
        /**
         * @brief Get the Z axis
         * 
         * @return Eigen::Vector3d the Z axis
         */
        static Eigen::Vector3d AxisZ() { return Eigen::Vector3d(0, 0, 1); }
    };

    /// A struct to store a plane
    struct TSPlane
    {
        TSPlane() {};
        /**
         * @brief Construct a new TSPlane object with the euclidean equation of a plane
         * 
         * @param a the a coefficient of the plane equation
         * @param b the b coefficient of the plane equation
         * @param c the c coefficient of the plane equation
         * @param d the d coefficient of the plane equation
         */
        TSPlane(double a, double b, double c, double d)
            : A(a), B(b), C(c), D(d), Normal(Eigen::Vector3d(a, b, c)), Center(Eigen::Vector3d(-a*d, -b*d, -c*d))
        {
            this->extractAxes();
        };
        /**
         * @brief Construct a new TSPlane object with a normal and a center
         * 
         * @param normal the normal of the plane
         * @param center the center of the plane
         */
        TSPlane(Eigen::Vector3d normal, Eigen::Vector3d center)
            : A(normal(0)), B(normal(1)), C(normal(2)), D(normal.dot(center)), Normal(normal), Center(center)
        {
            this->extractAxes();
        };
        /**
         * @brief Construct a new TSPlane object with 3 points
         * 
         * @param p1 the first point
         * @param p2 the second point
         * @param p3 the third point
         * @param placeholder a placeholder to distinguish this constructor from the one with 3 points and a normal
         */
        TSPlane(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, bool placeholder)
        {
            Eigen::Vector3d v1 = p2 - p1;
            this->AxisX = v1;
            Eigen::Vector3d v2 = p3 - p1;
            this->AxisY = v2;
            Eigen::Vector3d normal = v1.cross(v2);
            normal.normalize();
            this->A = normal(0);
            this->B = normal(1);
            this->C = normal(2);
            this->D = normal.dot(p1);
            this->Normal = normal;
            this->Center = p1;
        };
        /**
         * @brief Construct a plane passing through 2 points and a given normal
         * 
         * @param normal the normal of the plane
         * @param pt1 first point as center of the plane
         * @param pt2 second point to define the plane
         */
        TSPlane(Eigen::Vector3d normal, Eigen::Vector3d pt1, Eigen::Vector3d pt2)
        {
            Eigen::Vector3d v1 = pt2 - pt1;
            this->AxisX = v1;
            Eigen::Vector3d vec2 = normal.cross(v1);
            vec2.normalize();
            Eigen::Vector3d pt3 = pt1 + vec2;
            Eigen::Vector3d v2 = pt3 - pt1;
            this->AxisY = v2;
            
            Eigen::Vector3d normal2 = v1.cross(v2);
            normal2.normalize();
            this->A = normal2(0);
            this->B = normal2(1);
            this->C = normal2(2);
            this->D = normal2.dot(pt1);
            this->Normal = normal2;
            this->Center = pt1;

        };
        ~TSPlane() = default;
    
    public: __always_inline  ///< modified ops
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
    
    public: __always_inline  ///< custom funcs
        /**
         * @brief It computes the distance between a point and the plane
         * 
         * @param point the point to compute the distance
         * @return double the distance
         */
        double distance(Eigen::Vector3d point)
        {
            return std::fabs(this->Normal.dot(point - this->Center));
        };
        /**
         * @brief Compute the distance between two planes
         * 
         * @param plane the plane to compute the distance
         * @see distance(Eigen::Vector3d point)
         * @return double the distance
         */
        double distance(TSPlane plane)
        {
            return std::fabs(this->Normal.dot(plane.Center - this->Center));
        };
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

            // Eigen::Vector3d pt = point - this->distance(point) * this->Normal;
            // return (pt - point).norm();
        };
        /// Extract the two axes of the plane
        void extractAxes()
        {
            Eigen::Vector3d axisX, axisY;
            if (std::abs(this->Normal(0)) > std::abs(this->Normal(1)))
            {
                // Normal.x or Normal.z is the largest magnitude component, swap them
                double invLen = 1.0 / std::sqrt(this->Normal(0) * this->Normal(0) + this->Normal(2) * this->Normal(2));
                axisX(0) = -this->Normal(2) * invLen;
                axisX(1) = 0.0;
                axisX(2) = +this->Normal(0) * invLen;
            }
            else
            {
                // Normal.y or Normal.z is the largest magnitude component, swap them
                double invLen = 1.0 / std::sqrt(this->Normal(1) * this->Normal(1) + this->Normal(2) * this->Normal(2));
                axisX(0) = 0.0;
                axisX(1) = +this->Normal(2) * invLen;
                axisX(2) = -this->Normal(1) * invLen;
            }
            axisY = this->Normal.cross(axisX);
            this->AxisX = axisX;
            this->AxisY = axisY;
        }
        /// get closest points
        void getClosestPoints(std::vector<Eigen::Vector3d>& points,
                              std::vector<Eigen::Vector3d>& closestPoints,
                              std::vector<double>& distances)
        {
            for (auto& pt : points)
            {
                Eigen::Vector3d closestPt = this->projectPoint(pt);
                double dist = this->distance(pt);
                closestPoints.push_back(closestPt);
                distances.push_back(dist);
            }
        }

    public: __always_inline ///< transform funcs
        /**
         * @brief It computes the intersection between two planes
         * 
         * @param mat the transformation matrix (3x3)
         */
        void transform(Eigen::Matrix3d& mat)
        {
            Eigen::Vector3d normal = mat * this->Normal;
            Eigen::Vector3d center = mat * this->Center;
            this->Normal = normal;
            this->Center = center;
            this->A = normal(0);
            this->B = normal(1);
            this->C = normal(2);
            this->D = normal.dot(center);
        };
        /**
         * @brief To transform the plane with a 4x4 matrix
         * 
         * @param mat the transformation matrix (4x4)
         * @see transform(Eigen::Matrix3d& mat)
         */
        void transform(Eigen::Matrix4d& mat)
        {
            Eigen::Vector3d normal = mat.block<3, 3>(0, 0) * this->Normal;
            Eigen::Vector3d center = mat.block<3, 3>(0, 0) * this->Center + mat.block<3, 1>(0, 3);
            this->Normal = normal;
            this->Center = center;
            this->A = normal(0);
            this->B = normal(1);
            this->C = normal(2);
            this->D = normal.dot(center);
        };

    public: __always_inline  ///< static funcs
        /** 
         * @brief It checks if there is intersection between a ray and a plane following the:
         * Plane: ax+by+cz=d
         * Ray: P(t) = P0 + t * D
         * Intersection: P(t) = P0 + t * D = (x,y,z) = (a,b,c) * t = (a,b,c) * (d - (a*x+b*y+c*z)) / (a*a+b*b+c*c)
         * t = (d - (a*x+b*y+c*z)) / (a*a+b*b+c*c)
         * 
         * @see the function is modified from: https://asawicki.info/news_1428_finding_polygon_of_plane-aabb_intersection
         * 
         * @param RayOrig[out] the origin of the ray
         * @param RayDir[out] the direction of the ray
         * @param Plane[out] the plane to check the intersection with
         * @param OutT[in] the distance from the ray origin to the intersection point
         * @param OutVD[in] the distance from the ray origin to the plane
         * 
         * @return true if there is intersection
         * @return false if there is no intersection
         */
        static bool ray2PlaneIntersection(const Eigen::Vector3d &RayOrig,
                                          const Eigen::Vector3d &RayDir,
                                          const TSPlane &Plane,
                                          float *OutT,
                                          float *OutVD)
        {
            const Eigen::Vector3d PlaneNormal(Plane.A, Plane.B, Plane.C);

            const double Denominator = PlaneNormal.dot(RayDir);
            if (Denominator == 0.0f)  return false;

            const double Numerator = Plane.D - PlaneNormal.dot(RayOrig);
            const double t = Numerator / Denominator;

            if (OutT) *OutT = t;
            if (OutVD) *OutVD = Denominator;

            return true;
        }
        /** 
         * @brief It computes the intersection points between a plane and an AABB
         * 
         * @see the function is modified from: https://asawicki.info/news_1428_finding_polygon_of_plane-aabb_intersection
         * 
         * @param plane[out] the plane to check the intersection with
         * @param aabb_min[out] the minimum point of the AABB
         * @param aabb_max[out] the maximum point of the AABB
         * @param out_points[in] the intersection points
         * @param out_point_count[in] the number of intersection points (min:3, max: 6)
         */
        static void plane2AABBSegmentIntersect(const TSPlane &plane,
                                               const Eigen::Vector3d &aabb_min, 
                                               const Eigen::Vector3d &aabb_max,
                                               Eigen::Vector3d* out_points,
                                               unsigned &out_point_count)
        {
            out_point_count = 0;
            float vd, t;

            // Test edges along X axis, pointing right
            Eigen::Vector3d dir = Eigen::Vector3d(aabb_max[0] - aabb_min[0], 0.f, 0.f);
            Eigen::Vector3d orig = aabb_min;
            if (TSPlane::ray2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
                out_points[out_point_count++] = orig + dir * t;
            orig = Eigen::Vector3d(aabb_min[0], aabb_max[1], aabb_min[2]);
            if (TSPlane::ray2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
                out_points[out_point_count++] = orig + dir * t;
            orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_max[2]);
            if (TSPlane::ray2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
                out_points[out_point_count++] = orig + dir * t;
            orig = Eigen::Vector3d(aabb_min[0], aabb_max[1], aabb_max[2]);
            if (TSPlane::ray2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
                out_points[out_point_count++] = orig + dir * t;

            // Test edges along Y axis, pointing up
            dir = Eigen::Vector3d(0.f, aabb_max[1] - aabb_min[1], 0.f);
            orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_min[2]);
            if (TSPlane::ray2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
                out_points[out_point_count++] = orig + dir * t;
            orig = Eigen::Vector3d(aabb_max[0], aabb_min[1], aabb_min[2]);
            if (TSPlane::ray2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
                out_points[out_point_count++] = orig + dir * t;
            orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_max[2]);
            if (TSPlane::ray2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
                out_points[out_point_count++] = orig + dir * t;
            orig = Eigen::Vector3d(aabb_max[0], aabb_min[1], aabb_max[2]);
            if (TSPlane::ray2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
                out_points[out_point_count++] = orig + dir * t;

            // Test edges along Z axis, pointing forward
            dir = Eigen::Vector3d(0.f, 0.f, aabb_max[2] - aabb_min[2]);
            orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_min[2]);
            if (TSPlane::ray2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
                out_points[out_point_count++] = orig + dir * t;
            orig = Eigen::Vector3d(aabb_max[0], aabb_min[1], aabb_min[2]);
            if (TSPlane::ray2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
                out_points[out_point_count++] = orig + dir * t;
            orig = Eigen::Vector3d(aabb_min[0], aabb_max[1], aabb_min[2]);
            if (TSPlane::ray2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
                out_points[out_point_count++] = orig + dir * t;
            orig = Eigen::Vector3d(aabb_max[0], aabb_max[1], aabb_min[2]);
            if (TSPlane::ray2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
                out_points[out_point_count++] = orig + dir * t;

            // Test the 8 vertices
            orig = aabb_min;
            if (plane.A * orig.x() + plane.B * orig.y() + plane.C * orig.z() + plane.D == 0.f)
                out_points[out_point_count++] = orig;
        }
        /**
         * @brief Get the matrix (4x4) describing a plane to XY plane transform
         * 
         * @param source the starting plane
         * @return Eigen::Matrix4d the transform matrix (4x4)
         */
        static Eigen::Matrix4d getPlane2XYPlaneRotation(TSPlane& source)
        {
            Eigen::Vector3d O0 = source.Center;
            Eigen::Vector3d X0 = source.AxisX.normalized();
            Eigen::Vector3d Y0 = source.AxisY.normalized();
            Eigen::Vector3d Z0 = source.Normal.normalized();

            Eigen::Vector3d O1 = Eigen::Vector3d(0, 0, 0);
            Eigen::Vector3d X1 = Eigen::Vector3d(1, 0, 0);
            Eigen::Vector3d Y1 = Eigen::Vector3d(0, 1, 0);
            Eigen::Vector3d Z1 = Eigen::Vector3d(0, 0, 1);

            Eigen::Matrix3d F0;
            F0 << X0.x(), X0.y(), X0.z(),
                  Y0.x(), Y0.y(), Y0.z(),
                  Z0.x(), Z0.y(), Z0.z();
            Eigen::Matrix3d F1;
            F1 << X1.x(), Y1.x(), Z1.x(),
                  X1.y(), Y1.y(), Z1.y(),
                  X1.z(), Y1.z(), Z1.z();
            Eigen::Matrix3d F = F1 * F0;

            Eigen::Vector3d T0(0 - O0.x(), 0 - O0.y(), 0 - O0.z());
            Eigen::Vector3d T1(O1.x() - 0, O1.y() - 0, O1.z() - 0);

            Eigen::Matrix4d A;
            A.setIdentity();
            A.block<3, 1>(0, 3) = T0;

            Eigen::Matrix4d B;
            B.setIdentity();
            B.block<3, 3>(0, 0) = F;

            Eigen::Matrix4d C;
            C.setIdentity();
            C.block<3, 1>(0, 3) = T1;

            return C * B * A;
        };

    public: __always_inline  ///< bool funcs
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

    public:  ///< class members
        Eigen::Vector3d Normal;
        Eigen::Vector3d AxisX, AxisY;
        Eigen::Vector3d Center;
        double A, B, C, D;  ///< eq: ax+by+cz=d
    };

    /// A struct to store a segment object
    struct TSSegment
    {
        TSSegment() = default;
        TSSegment(Eigen::Vector3d p1, Eigen::Vector3d p2)
            : P1(p1), P2(p2)
        {};
        ~TSSegment() = default;
    
    public: __always_inline  ///< modified ops
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

    public: __always_inline  ///< bool checks
        /// it checks if a point is on a segment, also when the point is on the start or end of the segment
        bool isPointOnSegment(Eigen::Vector3d point) const
        {
            double AB = sqrt(pow(this->P2(0) - this->P1(0), 2) + pow(this->P2(1) - this->P1(1), 2) + pow(this->P2(2) - this->P1(2), 2));
            double AP = sqrt(pow(point(0) - this->P1(0), 2) + pow(point(1) - this->P1(1), 2) + pow(point(2) - this->P1(2), 2));
            double PB = sqrt(pow(this->P2(0) - point(0), 2) + pow(this->P2(1) - point(1), 2) + pow(this->P2(2) - point(2), 2));

            return (abs(AB - (AP + PB)) < 1e-5);
        };
        /**
         * @brief Test if the segment is connected to another segment
         * 
         * @param other the other segment to test
         * @return true if the segments are connected
         * @return false if the segments are not connected
         */
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

    public: __always_inline  ///< getters
        Eigen::Vector3d getDirection() const { return (this->P2 - this->P1).normalized(); };
        Eigen::Vector3d getCenter() const { return (this->P1 + this->P2) / 2.0; };
        Eigen::Vector3d getMidPoint()
        {
            Eigen::Vector3d midPoint = Eigen::Vector3d::Zero();
            midPoint = (this->P1 + this->P2) / 2.0;
            return midPoint;
        };

    public: __always_inline  ///< transform
        /**
        * @brief It transforms the segment with a given transform matrix (3x3)
        * 
        * @param transform the transform matrix
        */
        void transform(Eigen::Matrix3d& transform)
        {
            this->P1 = transform * this->P1;
            this->P2 = transform * this->P2;
        };
        /**
         * @brief It transforms the segment with a given transform matrix (4x4)
         * 
         * @param transform the transform matrix
         * @see transform(Eigen::Matrix3d& transform)
         */
        void transform(Eigen::Matrix4d& transform)
        {
            this->P1 = transform.block<3, 3>(0, 0) * this->P1 + transform.block<3, 1>(0, 3);
            this->P2 = transform.block<3, 3>(0, 0) * this->P2 + transform.block<3, 1>(0, 3);
        };

    public: __always_inline  ///< custom funcs
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
        /** 
         * @brief It intersects two segments and returns the intersection point
         * 
         * @param other[in] the other segment
         * @see intersect(const TSSegment& other, Eigen::Vector3d& intersection) const
         */
        bool intersect(const TSSegment& other) const
        {
            Eigen::Vector3d tempIntersectPt;
            return this->intersect(other, tempIntersectPt);
        };
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
            : m_Vertices(points), m_LinkedPlane(linkedPlane)
        {
            this->compute();
        };
        ~TSPolygon() = default;

    public: __always_inline   ///< modified ops
        bool operator==(const TSPolygon& other) const
        {
            ///< (a) check for number of vertices
            if (this->m_Vertices.size() == other.m_Vertices.size())
            {
                ///< (b) check for center (if the center is the same, the polygon is the same
                if (this->m_Center.isApprox(other.m_Center, 1e-5))
                    return true;
                
                ///< (c) check for vertices' coordinates
                uint NFoundPts = 0;
                for (uint i = 0; i < this->m_Vertices.size(); i++)
                {
                    for (uint j = 0; j < other.m_Vertices.size(); j++)
                    {
                        if (this->m_Vertices[i].isApprox(other.m_Vertices[j], 1e-5))
                            NFoundPts++;
                    }
                }
                if (NFoundPts == other.m_Vertices.size())
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
        TSSegment& operator[](uint i) {return m_Segments[i]; };

    private: __always_inline  ///< compute properties
        void compute() override
        {
            this->computeVertices();
            this->computeCenter();
            this->computeSegments();
            this->computeArea();
        };
        void computeVertices()
        {
            if (m_Vertices.size() != 0)
                return;
            else if (m_Vertices.size() != 0 && m_Segments.size() != 0)
            {
                for (auto s : m_Segments)
                {
                    bool found = false;
                    for (auto p : m_Vertices)
                        if (p == s.P1)
                        {
                            found = true;
                            break;
                        }
                    if (!found)
                        m_Vertices.push_back(s.P1);
                }
            }
        };
        void computeCenter()
        {
            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (auto p : m_Vertices)
                center += p;
            center /= m_Vertices.size();
            m_Center = center;
        };
        void computeSegments()
        {
            m_Segments.clear();

            /// (a) create segments
            for (uint i = 0; i < m_Vertices.size(); i++)
            {
                uint j = (i + 1) % m_Vertices.size();
                m_Segments.push_back(TSSegment(m_Vertices[i], m_Vertices[j]));
            }

            /// (b) for connected segments with the same direction, merge them
            bool merged = true;
            while (merged)
            {
                merged = false;
                for (uint i = 0; i < m_Segments.size(); i++)
                {
                    for (uint j = i + 1; j < m_Segments.size(); j++)
                    {
                        if (m_Segments[i].getDirection().isApprox(m_Segments[j].getDirection(), 1e-5) &&
                            m_Segments[i].P2.isApprox(m_Segments[j].P1, 1e-5))
                        {
                            m_Segments[i].P2 = m_Segments[j].P2;
                            m_Segments.erase(m_Segments.begin() + j);
                            merged = true;
                            break;
                        }
                    }
                    if (merged) break;
                }
            }

            this->computeCenter();
        }
        /// Compute the area of the polygon
        void computeArea()
        {
            double area = 0;
            for (uint i = 0; i < m_Vertices.size(); i++)
            {
                uint j = (i + 1) % m_Vertices.size();
                area += m_Vertices[i].x() * m_Vertices[j].y() - m_Vertices[j].x() * m_Vertices[i].y();
            }
            this->m_Area = area;
        };

    public: __always_inline  ///< vertices ops
        void addVertex(Eigen::Vector3d point) {m_Vertices.push_back(point); compute(); };
        void removeVertex(uint i) {m_Vertices.erase(m_Vertices.begin() + i); compute(); };
        void setVertices(std::vector<Eigen::Vector3d> points) {m_Vertices = points; compute(); };
        void setLinkedPlane(TSPlane linkedPlane) {m_LinkedPlane = linkedPlane; };

    public:__always_inline  ///< getters
        std::vector<Eigen::Vector3d>& getVertices() {return m_Vertices; };
        uint getNumVertices() {return m_Vertices.size(); };
        Eigen::Vector3d& getVertex(uint i) {return m_Vertices[i]; };
        int getVertexIndex(Eigen::Vector3d point)
        {
            for (uint i = 0; i < m_Vertices.size(); i++)
                if (m_Vertices[i].isApprox(point))
                    return i;
            return -1;
        };
        Eigen::Vector3d& getCenter() {return m_Center; };
        Eigen::Vector3d getNormal() {return m_LinkedPlane.Normal; };
        /**
         * @brief Get the Linked Plane object
         * 
         * @return TSPlane& it returns the plane on which the polygon lies
         */
        TSPlane& getLinkedPlane() {return m_LinkedPlane; };
        /**
         * @brief Get the Segments object
         * 
         * @return std::vector<TSSegment>& it returns the polygon's segments
         */
        std::vector<TSSegment>& getSegments() {return m_Segments; };
        double getArea() {return m_Area; };
        uint size() {return m_Segments.size(); };

    public: __always_inline  ///< condition checks
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
            if (m_Vertices.size() < 3)
                return false;
            
            for (uint i = 0; i < m_Vertices.size(); i++)
            {
                uint j = (i + 1) % m_Vertices.size();
                if (!m_Segments[i].isConnected(m_Segments[j]))
                    return false;
            }

            if (this->getArea() < 1e-5)
                return false;

            return true;
        };
        /**
         * @brief Check if a point is inside the polygon by checking if the point is on the same side of all the polygon's segments.
         * 
         * @ref we implemented a 3D version of the code present here:
         *      https://inginious.org/course/competitive-programming/geometry-pointinconvex
         * 
         * @param point the point to check
         * @return true if the point is inside the polygon
         * @return false if the point is not inside the polygon
         */
        bool isPointInsidePolygon(Eigen::Vector3d point)
        {
            bool isInside = true;
            for (uint k = 0; k < this->getVertices().size(); k++)
            {
                Eigen::Vector3d v1 = this->getVertices()[k];
                Eigen::Vector3d v2 = this->getVertices()[(k + 1) % this->getVertices().size()];

                Eigen::Vector3d v1v2 = v2 - v1;
                Eigen::Vector3d v1ctr = point - v1;

                Eigen::Vector3d cross = v1v2.cross(v1ctr);

                Eigen::Vector3d polyPlnNormal = this->getNormal();

                if (cross.dot(polyPlnNormal) < 0)
                {
                    isInside = false;
                    break;
                }
            }

            return isInside;
        };
        /// Check if the polygon is a quadrilateral
        bool isQuadrilateral()
        {
            if (m_Vertices.size() == 4)
                return true;
            return false;
        };
        /// Compare two polygons centers and return true if they are equal
        bool areCentersEqual(TSPolygon& other, float eps = 1e-5)
        {
            return this->m_Center.isApprox(other.m_Center, eps);
        };

    public: __always_inline  ///< transform funcs
        /**
         * @brief It transforms the polygon by an identity matrix (3x3)
         * 
         * @param mat the identity matrix
         */
        void transform(Eigen::Matrix3d& mat)
        {
            for (auto& v : this->m_Vertices)
            {
                v = mat * v;
            }

            this->m_LinkedPlane.transform(mat);

            this->compute();
        };
        /**
         * @brief It transforms the polygon by a 4x4 matrix
         * 
         * @param mat the 4x4 matrix
         * @see transform(Eigen::Matrix3d& mat)
         */
        void transform(Eigen::Matrix4d& mat)
        {
            for (auto& v : this->m_Vertices)
                v = mat.block<3, 3>(0, 0) * v + mat.block<3, 1>(0, 3);

            this->m_LinkedPlane.transform(mat);

            this->compute();
        };
        /**
         * @brief It rotates the polygon around the X axis
         * 
         * @param angle the rotation angle
         * @return Eigen::Matrix3d the rotation matrix
         */
        Eigen::Matrix3d rotateByAxisX(double angle)
        {
            Eigen::Matrix3d rotMat;
            rotMat = Eigen::AngleAxis(angle, Eigen::Vector3d::UnitX());
            this->transform(rotMat);
            return rotMat;
        };
        /**
         * @brief It rotates the polygon around the Y axis
         * 
         * @param angle the rotation angle
         * @return Eigen::Matrix3d the rotation matrix
         */
        Eigen::Matrix3d rotateByAxisY(double angle)
        {
            Eigen::Matrix3d rotMat;
            rotMat = Eigen::AngleAxis(angle, Eigen::Vector3d::UnitY());
            this->transform(rotMat);
            return rotMat;
        };
        /**
         * @brief It rotates the polygon around the Z axis
         * 
         * @param angle the rotation angle
         * @return Eigen::Matrix3d the rotation matrix
         */
        Eigen::Matrix3d rotateByAxisZ(double angle)
        {
            Eigen::Matrix3d rotMat;
            rotMat = Eigen::AngleAxis(angle, Eigen::Vector3d::UnitZ());
            this->transform(rotMat);
            return rotMat;
        };

    public: __always_inline  ///< custom funcs
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
            for (auto& v : this->m_Vertices)
            {
                if (v.isApprox(segment.P1) || v.isApprox(segment.P2))
                    return false;
            }

            ///< (1.b) catch the case where both of the segment's end points are not on the polygon's contour
            if (!this->isPointOnPolygon(segment.P1) && !this->isPointOnPolygon(segment.P2))
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
            else if (!this->isPointOnPolygon(segment.P1) || !this->isPointOnPolygon(segment.P2))
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

            for (uint i = 0; i < m_Vertices.size(); i++)
            {
                TSSegment testSeg = TSSegment(segment.P1, m_Vertices[i]);
                double angle = segment.angleDegOnPlane(testSeg, m_LinkedPlane);

                if (angle < 0)
                    pointsA.push_back(m_Vertices[i]);
                else if (angle > 0)
                    pointsB.push_back(m_Vertices[i]);
                else
                {
                    if (segment.isPointOnSegment(m_Vertices[i]))
                    {
                        pointsA.push_back(m_Vertices[i]);
                        pointsB.push_back(m_Vertices[i]);
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
        // FIXME: in case the polygon is still orthogonal to the global coordinate system after the rotation, 
        // the code needs to be reworked
        /**
         * @brief Reorder the polygon vertices in a clockwise order based on graham scan algorithm.
         *      The geometry is first rotated by a small angle to avoid the case where the plane is 
         *      orthogonal to the global coordinate system. It might still possible that after the
         *      rotation the plane is still orthogonal to the global coordinate system. In this case
         *      a warning is printed and code needs to be reworked.
         * 
         */
        void reorderClockwisePoints()
        {
            // rotate to avoid corner case where the plane is orthogonal to the global coordinate system
            double rotAngle = 20;
            Eigen::Matrix3d matRotX = this->rotateByAxisX(rotAngle);
            Eigen::Matrix3d matRotY = this->rotateByAxisY(rotAngle);
            Eigen::Matrix3d matRotZ = this->rotateByAxisZ(rotAngle);

            // apply grham scanning
            std::sort(m_Vertices.begin(),m_Vertices.end(),
                  [&](const Eigen::Vector3d& a, 
                  const Eigen::Vector3d& b)
            {
                Eigen::Vector3d a_center = a - this->m_Center;
                Eigen::Vector3d b_center = b - this->m_Center;
                float angleA = std::atan2(a_center(1) * m_LinkedPlane.A - a_center(0) * m_LinkedPlane.B,
                                            a_center(0) * m_LinkedPlane.A + a_center(1) * m_LinkedPlane.B);
                float angleB = std::atan2(b_center(1) * m_LinkedPlane.A - b_center(0) * m_LinkedPlane.B,
                                            b_center(0) * m_LinkedPlane.A + b_center(1) * m_LinkedPlane.B);
                return angleA < angleB;
            });

            // rotate back
            this->rotateByAxisZ(-rotAngle);
            this->rotateByAxisY(-rotAngle);
            this->rotateByAxisX(-rotAngle);

            // recompute the polygon properties
            this->compute();

            return;
        };
        /**
         * @brief It triangulate a convex polygon by groups of 3 vertices and alawys the first index as 0.
         *                  A
         *                  x
         *        O'       t3       B
         *         o _____________ x
         *           \             
         *            \           
         *             \    t2   
         *              \       
         *       D x  t1 \     
         *                \   
         *                  x
         *                  C
         * 
         * @param polyVertices[in] the polygon's vertices
         * @param polyTriangles[out] the triangles' indices
         */
        void triangulate(std::vector<Eigen::Vector3d>& polyVertices,
                         std::vector<Eigen::Vector3i>& polyTriangles)
        {
            Eigen::Vector3i triangle;

            if (this->getNumVertices() == 3)
            {
                triangle(0) = 0;
                triangle(1) = 1;
                triangle(2) = 2;
                polyTriangles.push_back(triangle);
            }
            else
            {
                for  (unsigned i = 0; i < this->getNumVertices() - 2; i += 1)
                {
                    triangle(0) = 0;
                    triangle(1) = (i + 1) % this->getNumVertices();
                    triangle(2) = (i + 2) % this->getNumVertices();
                    polyTriangles.push_back(triangle);
                }
            }
        };

    private:
        /// The polygon's vertices
        std::vector<Eigen::Vector3d> m_Vertices;
        /// The polygon's center
        Eigen::Vector3d m_Center;
        /// The plane on which the polygon lies
        TSPlane m_LinkedPlane;
        /// The polygon's segments
        std::vector<TSSegment> m_Segments;
        /// the polygon's area
        double m_Area;
    };

}