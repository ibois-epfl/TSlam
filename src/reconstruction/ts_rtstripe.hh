#pragma once

#include "ts_compute.hh"
#include "ts_rtag.hh"

#include <array>


namespace tslam
{
    class TSRTStripe : public TSCompute
    {
    public:
        TSRTStripe(std::vector<TSRTag> tags);
        ~TSRTStripe() = default;

        inline int size() {return m_Tags.size(); };

        /// Reorde the tags based on the axis of the stripe (longest edge)
        void reorderTags();

        inline double getLength() {return m_Length; };

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
        

    private:
        std::vector<TSRTag> m_Tags;
        double m_Length;
        Eigen::Vector3d m_AxisX;
        Eigen::Vector3d m_AxisY;
        Eigen::Vector3d m_Normal;
    };
}