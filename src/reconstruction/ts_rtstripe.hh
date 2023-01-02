#pragma once

#include "ts_compute.hh"
#include "ts_rtag.hh"
#include "ts_geo_util.hh"

#include <array>


namespace tslam
{
    /// Stripe of tags
    class TSRTStripe : public TSCompute
    {
    public:
        TSRTStripe() = default;
        TSRTStripe(std::vector<TSRTag> tags);
        ~TSRTStripe() = default;

        inline int size() {return m_Tags.size(); };

        /// Reorde the tags based on the axis of the stripe (longest edge)
        void reorderTags();

    public: __always_inline  ///< Accessors
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