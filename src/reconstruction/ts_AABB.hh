#pragma once

#include <CGAL/Simple_cartesian.h>
#include <CGAL/bounding_box.h>
#include <CGAL/Iso_cuboid_3.h>

#include <Eigen/Core>

#include <vector>
#include <iostream>
#include <tuple>

typedef double                          FT;
typedef CGAL::Simple_cartesian<FT>      K_AABB;
typedef K_AABB::Point_3                 Point_3_AABB;
typedef K_AABB::Iso_cuboid_3            AABB;

namespace tslam::Reconstruction
{
    /**
     * @brief TSAABB is a class that represents an axis-aligned bounding box (AABB) in CGAL 
     * and provides methods to compute it and to scale it.
     * 
     */
    class TSAABB
    {
    public:
        TSAABB() = default;
        /**
         * @brief Construct a new TSAABB object
         * 
         * @param points the list of points to be used to compute the AABB
         */
        TSAABB(std::vector<Eigen::Vector3d> points)
        {
            std::vector<Point_3_AABB> points_3;
            for (auto& pt : points)
                points_3.push_back(Point_3_AABB(pt(0), pt(1), pt(2)));
            this->m_Points = points_3;

            this->computeAABB();
        };
        ~TSAABB() = default;

    private: __attribute__((always_inline))
        /// It computes the AABB given a list of points
        void computeAABB()
        {
            this->m_AABB = CGAL::bounding_box(this->m_Points.begin(),
                                              this->m_Points.end());
            this->m_Min = Eigen::Vector3d(this->m_AABB.xmin(),
                                          this->m_AABB.ymin(),
                                          this->m_AABB.zmin());
            this->m_Max = Eigen::Vector3d(this->m_AABB.xmax(),
                                          this->m_AABB.ymax(),
                                          this->m_AABB.zmax());
        };

    public: __attribute__((always_inline))  ///< transform
        /// Uniform scaling
        void scale(double scaleFactor)
        {
            // find the vector between the min and max points
            Eigen::Vector3d min = this->getMin();
            Eigen::Vector3d max = this->getMax();
            Eigen::Vector3d diff = max - min;

            // extend the position of both min and max points by the difference vector 
            // multiplied by the scale factor
            min -= diff * scaleFactor;
            max += diff * scaleFactor;

            // set the new min and max points
            this->m_Min = min;
            this->m_Max = max;
        };

    public: __attribute__((always_inline))  ///< getters
        Eigen::Vector3d& getMin() {return this->m_Min; };
        Eigen::Vector3d& getMax() {return this->m_Max; };
        
        AABB getAABB() {return this->m_AABB; };

        private:  ///< private members
            std::vector<Point_3_AABB> m_Points;
            AABB m_AABB;
            Eigen::Vector3d m_Min;
            Eigen::Vector3d m_Max;
    };
}
