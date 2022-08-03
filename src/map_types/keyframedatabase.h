/**
* This file is part of  TSLAM
*
* Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)
*
* TSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* TSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with TSLAM. If not, see <http://wwmap->gnu.org/licenses/>.
*/
#ifndef TSLAM_FrameDataBase_H
#define TSLAM_FrameDataBase_H
#include <iostream>
#include <memory>
#include <vector>
#include <set>
#include "tslam_exports.h"
namespace tslam{
class KFDataBaseVirtual;
class Frame;
class FrameSet;
class CovisGraph;

class TSLAM_API KeyFrameDataBase{
public:
    KeyFrameDataBase();
    void loadFromFile(const std::string &filePathOrNothing);
    bool isEmpty()const;
    bool isId(uint32_t id)const ;

    bool add(Frame &f);
    bool del(const Frame &f);
    void clear();

    size_t size()const;
    void toStream(std::iostream &str)const ;
    void fromStream(std::istream &str);
    std::vector<uint32_t> relocalizationCandidates(Frame &frame, FrameSet &fset, CovisGraph &covisgraph, bool sorted=true, float minScore=0, const std::set<uint32_t> &excludedFrames={});
    float score(Frame &f,Frame &f2);

    uint64_t getSignature()const;
private:
    std::shared_ptr<KFDataBaseVirtual> _impl;
    int _type=-1;
};
}
#endif
