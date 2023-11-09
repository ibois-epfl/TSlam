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
#ifndef _tslam_MinMaxBags_H
#define _tslam_MinMaxBags_H
#include "heap.h"
namespace tslam{
/** Class that represent where you can only add X elements and only these with min values are kept.
 * If the number of elements introduced is greater than the limit, the bag only kepts the lowest values
 */
template<typename T>
class MinBag{
    size_t _maxSize=0;
    tslam::Heap<T,std::greater<T>> heap;
public:

    MinBag(int maxSize=-1) {setMaxSize(maxSize);}
    void setMaxSize(int maxSize) {_maxSize=maxSize;}
    inline void push(const T  &v){
        assert(_maxSize>0);
        if( heap.size()>=_maxSize){
            if ( heap.array[0] > v) {
                heap.pop();
                heap.push(v);
            }
        }
        else
            heap.push(v);
    }
    inline T pop(){return heap.pop();}
    inline bool empty()const{return heap.empty();}
};

/** Class that represent where you can only add X elements and only these with max values are kept.
 * If the number of elements introduced is greater than the limit, the bag only kepts the highest values
 */
template<typename T>
class MaxBag{
    size_t _maxSize=0;
    tslam::Heap<T,std::less<T>> heap;
public:

    MaxBag(int maxSize=-1) {setMaxSize(maxSize);}
    void setMaxSize(int maxSize) {_maxSize=maxSize;}
    inline void push(const T  &v){
        assert(_maxSize>0);
        if( heap.size()>= _maxSize){
            if ( heap.array[0] < v) {
                heap.pop();
                heap.push(v);
            }
        }
        else
            heap.push(v);
    }
    inline T pop(){return heap.pop();}
    inline bool empty()const{return heap.empty();}
};
}

#endif
