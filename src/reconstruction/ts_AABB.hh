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
