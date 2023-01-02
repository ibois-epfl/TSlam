#include "ts_rtstripe.hh"

#include <algorithm>
#include <functional>
#include <iostream>
#include <string_view>

namespace tslam
{
    TSRTStripe::TSRTStripe(std::vector<TSRTag> tags)
    {
        this->m_Tags = tags;
        // this->reorderTags();
        this->compute();
    };

    void TSRTStripe::compute()
    {
        this->m_Length = this->computeLength();
        this->m_Normal = this->computeAverageNormal();
        this->m_AxisX = this->computeAverageXAxis();
        this->m_AxisY = this->computeAverageYAxis();
        this->m_MeanPlane = this->computeMeanPlane();
    };
    double TSRTStripe::computeLength()
    {
        double length = 0;
        for (auto& tag : this->m_Tags)
        {
            double dist = tag.distance2Pt(this->m_Tags[0].getCenter());
            if (dist > length)
                length = dist;
        }
        return length;
    };
    Eigen::Vector3d TSRTStripe::computeAverageNormal()
    {
        Eigen::Vector3d normal = Eigen::Vector3d::Zero();
        for (auto& tag : this->m_Tags)
            normal += tag.getNormal();
        normal /= this->m_Tags.size();
        return normal;
    };
    Eigen::Vector3d TSRTStripe::computeAverageXAxis()
    {
        Eigen::Vector3d axis = Eigen::Vector3d::Zero();
        for (auto& tag : this->m_Tags)
            axis += tag.getAxisX();
        axis /= this->m_Tags.size();
        return axis;
    };
    Eigen::Vector3d TSRTStripe::computeAverageYAxis()
    {
        Eigen::Vector3d axis = Eigen::Vector3d::Zero();
        for (auto& tag : this->m_Tags)
            axis += tag.getAxisY();
        axis /= this->m_Tags.size();
        return axis;
    };
    TSPlane TSRTStripe::computeMeanPlane()
    {
        return TSPlane(this->getNormal(),
                       this->front().getCenter(),
                       this->back().getCenter());
    };

    void TSRTStripe::reorderTags()
    {
        // determine reference point on the x axis direction out of the stripe length
        Eigen::Vector3d ptRef = Eigen::Vector3d::Zero();
        ptRef = this->m_Tags[0].getCenter();
        ptRef = ptRef + this->m_Length * 10000 * m_Tags[0].getAxisX();

        // sort the tags based on the distance to the reference point
        std::sort(this->m_Tags.begin(), this->m_Tags.end(), [ptRef] (TSRTag& a, TSRTag& b) {
            return (a.distance2Pt(ptRef) < b.distance2Pt(ptRef));
        });

        // set the flag of the extremity tags
        this->m_Tags.front().setType(TSRTagType::Edge);
        this->m_Tags.back().setType(TSRTagType::Edge);

        // recompute everythin now that tags are ordered
        this->compute();
    };
}