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
#ifndef _XFLANN_HHEAP_
#define _XFLANN_HHEAP_
#include <iostream>
#include <cassert>
#include <vector>
#include <functional>

namespace tslam{
//heap having at the top the minimum element
template<typename T, class comparison=std::less<T>,typename Container=std::vector<T>>
class Heap{
  public:
    Container array;
    comparison comp;
public:
    Heap( ){}

    void reset(){
        array.reset();
    }
    void reserve(int size){array.reserve(size);}

    //returns true if the element has been added and false if not (the heap is full)
    inline void push (const T &val)
    {
        array.push_back(val);
        down ( array.size()-1) ;
    }


    inline T pop ( ){
        T res=array[0];
        std::swap(array[0],array[array.size()-1]);
        array.pop_back();
        if (array.size() > 1)  up (0) ;
        return res;
    }
    inline size_t size()const {return array.size();}

    inline bool empty()const {return array.size()==0;}
    const T& top()const{return array[0];}
private:


    inline void
    down ( size_t index)
    {
        if (index == 0) return  ;

        size_t parentIndex =(index - 1) / 2;

        if ( comp(array[ index], array[ parentIndex])) {
            std::swap (array[ index], array[parentIndex]) ;
            down (parentIndex) ;
        }
    }

    inline void up (size_t index)
    {
        size_t leftIndex  =   2 * index + 1;
        size_t rightIndex = 2 * index + 2  ;

        /* no childer: stop */
        if (leftIndex >= array.size()) return ;

        /* only left childer: easy */
        if (rightIndex >= array.size()) {
            if (comp(array[leftIndex], array [ index]) ) {
                std::swap (array[index], array[leftIndex]) ;
            }
            return ;
        }

        /* both childern */
        {
            if ( comp(array[  leftIndex] , array[ rightIndex])) {
                /* swap with left */
                if ( comp(array[leftIndex], array [index]) ) {
                    std::swap (array [index], array[leftIndex]) ;
                    up ( leftIndex) ;
                }
            } else {
                /* swap with right */
                if ( comp(array[rightIndex], array[ index])  ) {
                    std::swap (array [index], array[rightIndex]) ;
                    up ( rightIndex) ;
                }
            }
        }
    }


};



}


#endif
