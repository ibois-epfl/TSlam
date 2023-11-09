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
#include "ts_rtstripe.hh"

#include <algorithm>
#include <functional>
#include <iostream>
#include <string_view>

namespace tslam::Reconstruction
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
        if (this->m_Tags.size() > 1)
            return TSPlane(this->getNormal(),
                           this->front().getCenter(),
                           this->back().getCenter());
        else
            return TSPlane(this->getNormal(),
                           this->front().getCenter(),
                           this->front().getCenter() + this->front().getAxisX());
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

        // recompute everythin now that tags are ordered
        this->compute();
    };
}