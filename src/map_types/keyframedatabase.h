/**
This file is part of TSLAM and distributed under the terms of the GPLv3.

The modified version: Copyright (C) 2023 Andrea Settimi <andrea dot settimi at epfl dot ch>, Hong-Bin Yang <hong dash bin dot yang at epfl dot ch> (IBOIS, EPFL)
The original project: Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas, MODIFIED BY Andrea Settimi and Hong-Bin Yang ''AS IS'' AND ANY EXPRESS OR IMPLIED
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
or implied, of Rafael Muñoz Salinas.
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
