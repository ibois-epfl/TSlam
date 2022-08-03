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
