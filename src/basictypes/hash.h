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
#ifndef Hash_H_
#define Hash_H_
#include <cstdint>
#include <iostream>
#include <opencv2/core/core.hpp>
namespace tslam {
/**
 * @brief The Hash struct creates a hash by adding elements. It is used to check the integrity of data stored in files in debug mode
 */
struct Hash{
    uint64_t seed=0;

    template<typename T> void add(const T &val){
        char *p=(char *)&val;
        for(uint32_t b=0;b<sizeof(T);b++) seed  ^=  p[b]+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    template<typename T> void add(const  T& begin,const T& end){
        for(auto it=begin;it!=end;it++)add(*it);
    }

    void add(bool val){
        seed  ^=   int(val)+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    void add(int val){
        seed  ^=   val+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    void add(uint64_t val){
        seed  ^=   val+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    void add(int64_t val){
        seed  ^=   val+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    void add(float val){
        int *p=(int *)&val;
        seed  ^=   *p+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    void add(double val){
        uint64_t *p=(uint64_t *)&val;
        seed  ^=   *p+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    void add(const cv::Mat & m){
        for(int r=0;r<m.rows;r++){
            const char *ip=m.ptr<char>(r);
            int nem= m.elemSize()*m.cols;
            for(int i=0;i<nem;i++)
                seed  ^=   ip[i]+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
    }
    void add(const cv::Point2f & p){
        add(p.x);
        add(p.y);
    }
    void add(const cv::Point3f & p){
        add(p.x);
        add(p.y);
        add(p.z);
    }

    void operator+=(bool v){add(v);}
    void operator+=(int v){add(v);}
    void operator+=(char v){add(v);}
    void operator+=(float v){add(v);}
    void operator+=(double v){add(v);}
    void operator+=(uint32_t v){add(v);}
    void operator+=(uint64_t v){add(v);}
    void operator+=(int64_t v){add(v);}
    void operator+=(const cv::Mat & v){add(v);}
    void operator+=(const cv::Point2f & v){add(v);}
    void operator+=(const cv::Point3f & v){add(v);}
    void operator+=(const std::string & str){
        for(const auto &c:str)add(c);
    }


     operator uint64_t()const{return seed;}


    std::string tostring( ){
        return tostring(seed);
    }


    static std::string tostring(uint64_t v){
        std::string sret;
        std::string alpha="qwertyuiopasdfghjklzxcvbnm1234567890QWERTYUIOPASDFGHJKLZXCVBNM";
        unsigned char * s=(uchar *)&v;
        int n=sizeof(seed)/sizeof(uchar );
        for(int i=0;i<n;i++){
            sret.push_back(alpha[s[i]%alpha.size()]);
        }
        return sret;
    }
    void reset(){seed=0;}
};

}

#endif

