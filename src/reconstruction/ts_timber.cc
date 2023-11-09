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
#include "ts_timber.hh"


namespace tslam::Reconstruction
{
    TSTimber::TSTimber(std::vector<TSRTag> planeTags)
        : m_RTags(planeTags)
    {
        this->compute();
    }

    void TSTimber::scaleAABB(double scale)
    {
        this->m_AABB.scale(scale);
    }

    void TSTimber::removeDuplicateTags(double distancethreshold)
    {
        std::vector<TSRTag> tagsNoDuplTemp;
        std::vector<Eigen::Vector3d> ctrs;

        for (auto& p : this->getPlaneTags())
            ctrs.push_back(p.getCenter());

        for (size_t i = 0; i < ctrs.size(); i++)
        {
            for (size_t j = i + 1; j < ctrs.size(); j++)
            {
                if (ctrs[i].isApprox(ctrs[j]))
                {
                    ctrs.erase(ctrs.begin() + j);
                }
            }
        }

        for (auto& p : ctrs)
        {
            for (auto& q : this->getPlaneTags())
            {
                if (p.isApprox(q.getCenter()))
                {
                    tagsNoDuplTemp.push_back(q);
                }
            }
        }

        this->m_RTags.clear();
        this->m_RTags = tagsNoDuplTemp;
    }

    void TSTimber::setPlaneTagsFromYAML(const std::string& filename)
    {
        TSRTag::parseFromMAPYAML(filename, this->m_RTags);
        this->removeDuplicateTags();
        this->compute();
    };

    void TSTimber::compute()
    {
        this->computeAABB();
        this->computeTagsCtrs();

    }
    void TSTimber::computeAABB()
    {
        std::vector<Eigen::Vector3d> pts;
        for (auto& p : this->getPlaneTags())
            pts.push_back(p.getCenter());

        this->m_AABB = TSAABB(pts);
    }
    
    void TSTimber::computeTagsCtrs()
    {
        this->m_RtagsCtrs.clear();
        for (auto& p : this->getPlaneTags())
        {
            this->m_RtagsCtrs.push_back(p.getCenter());
        }
    }
}