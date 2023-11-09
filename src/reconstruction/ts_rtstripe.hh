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
#include "ts_rtag.hh"
#include "ts_geo_util.hh"

#include <array>


namespace tslam::Reconstruction
{
    /// Stripe of tags
    class TSRTStripe : public TSCompute
    {
    public:
        TSRTStripe() = default;
        TSRTStripe(std::vector<TSRTag> tags);
        ~TSRTStripe() = default;

        inline int size() {return m_Tags.size(); };

        /// Reorder the tags based on the axis of the stripe (longest edge)
        void reorderTags();

    public: __attribute__((always_inline))  ///< Accessors
        double getLength() {return m_Length; };
        Eigen::Vector3d& getAxisX() {return m_AxisX; };
        Eigen::Vector3d& getAxisY() {return m_AxisY; };
        Eigen::Vector3d& getNormal() {return m_Normal; };
        TSPlane getMeanPlane() {return m_MeanPlane; };
        std::vector<TSRTag>& getTags() {return m_Tags; };

    public:  ///< Setters
        inline void setMeanPlane(TSPlane plane) {m_MeanPlane = plane; };

    public:  ///< Modified accessors / vector mutators
        inline void clear() {m_Tags.clear(); };
        inline void push_back(TSRTag& tag) {m_Tags.push_back(tag);};
        inline TSRTag& front() {return m_Tags.front(); };
        inline TSRTag& back() {return m_Tags.back(); };
        inline TSRTag& operator[](int i) {return m_Tags[i]; };
        inline friend std::ostream& operator<<(std::ostream& os, TSRTStripe& stripe)
        {
            os << "Stripe containing tags: " << std::endl;
            for (auto& tag : stripe.m_Tags)
                os << tag << std::endl;
            return os;
        };
        inline std::vector<TSRTag>::iterator begin() {return m_Tags.begin(); };
        inline std::vector<TSRTag>::iterator end() {return m_Tags.end(); };
        inline TSRTStripe& operator+=(TSRTStripe& stripe)
        {
            for (auto& tag : stripe.m_Tags)
                this->m_Tags.push_back(tag);
            this->reorderTags();
            return *this;
        };
    
    private:
        void compute() override;
        /// Compute the length of the stripe (longest edge)
        double computeLength();
        /// Average the normal of all the tags' normals in the stripe
        Eigen::Vector3d computeAverageNormal();
        /// Compute the x axis of the stripe (longest edge)
        Eigen::Vector3d computeAverageXAxis();
        /// Compute the y axis of the stripe (shortest edge)
        Eigen::Vector3d computeAverageYAxis();
        /// Compute the mean plane of the stripe
        TSPlane computeMeanPlane();

    private:
        /// The tags of the stripe
        std::vector<TSRTag> m_Tags = {};
        /// The length of the stripe (longest edge)
        double m_Length;
        /// The average normal, axis x and axis y of the stripe's tags
        Eigen::Vector3d m_Normal, m_AxisY, m_AxisX;
        /// The associated plane with the stripe passing through the extremes and the average normal
        TSPlane m_MeanPlane;
    };
}