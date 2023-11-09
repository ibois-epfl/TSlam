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
#ifndef TSLAM_TIMERS_H
#define TSLAM_TIMERS_H


#include <chrono>
#include <string>
#include <vector>
#include <iostream>
#include "debug.h"
namespace tslam{

//timer

struct ScopedTimerEvents
{
    enum SCALE {NSEC,MSEC,SEC};
    SCALE sc;
    std::vector<std::chrono::high_resolution_clock::time_point> vtimes;
    std::vector<std::string> names;
    std::string _name;

 inline   ScopedTimerEvents(const std::string &name="",bool start=true,SCALE _sc=MSEC){
         if(start)
             add("start");
         sc=_sc;
         _name=name;
     }

   inline void add(const std::string &name){
         vtimes.push_back(std::chrono::high_resolution_clock::now());
        names.push_back(name);
     }
    inline void addspaces(std::vector<std::string> &str ){
        //get max size
        size_t m=0;
        for(auto &s:str)m=(std::max)(size_t(s.size()),m);
        for(auto &s:str){
            while(s.size()<m) s.push_back(' ');
        }
    }

   inline ~ScopedTimerEvents(){
//         if (!debug::Debug::showTimer()) return;
        double fact=1;
        std::string str;
        switch(sc)
        {
        case NSEC:fact=1;str="ns";break;
        case MSEC:fact=1e6;str="ms";break;
        case SEC:fact=1e9;str="s";break;
        };

        add("total");
        addspaces(names);
        for(size_t i=1;i<vtimes.size();i++){
            std::cout<<"Time("<<_name<<")-"<<names[i]<<" "<< double(std::chrono::duration_cast<std::chrono::nanoseconds>(vtimes[i]-vtimes[i-1]).count())/fact<<str<<" "<<double(std::chrono::duration_cast<std::chrono::nanoseconds>(vtimes[i]-vtimes[0]).count())/fact<<str<<std::endl;
        }
     }
};

inline std::string methodName(  std::string  prettyFunction)
{
    std::string   res;
    res.reserve(prettyFunction.size());
    bool spaceFound=false;
    for(auto c:prettyFunction){
        if(c==' '  && !spaceFound)spaceFound=true;
        else if(c!='(' && spaceFound) res.push_back(c);
        else if (c=='(' &&spaceFound) break;
        }
    return res;
}

#ifdef USE_TIMERS

#define __TSLAM_ADDTIMER__ ScopedTimerEvents XTIMER_X(methodName(__PRETTY_FUNCTION__));
#define __TSLAM_TIMER_EVENT__(Y) XTIMER_X.add(Y);
#else
#define __TSLAM_ADDTIMER__
#define __TSLAM_TIMER_EVENT__(Y)
#endif

struct Timer{
    enum SCALE {NSEC,MSEC,SEC};

    std::chrono::high_resolution_clock::time_point _s;
    double sum=0,n=0;
    std::string _name;
    Timer(){}

    Timer(const std::string &name):_name(name){}
    void setName(std::string name){_name=name;}
   inline void start(){_s=std::chrono::high_resolution_clock::now();}
   inline void end()
    {
#ifdef USE_TIMERS
        auto e=std::chrono::high_resolution_clock::now();
        sum+=double(std::chrono::duration_cast<std::chrono::nanoseconds>(e-_s).count());
        n++;
#endif
    }

   inline void print(SCALE sc=MSEC){
#ifdef USE_TIMERS
       if (!debug::Debug::showTimer()) return;
        double fact=1;
        std::string str;
        switch(sc)
        {
        case NSEC:fact=1;str="ns";break;
        case MSEC:fact=1e6;str="ms";break;
        case SEC:fact=1e9;str="s";break;
        };
        std::cout<<"Time("<<_name<<")= "<< ( sum/n)/fact<<str<<std::endl;
#endif
    }

};


struct TimerAvrg{
    std::vector<double> times;
    size_t curr=0,n;
    std::chrono::high_resolution_clock::time_point begin,end;

    TimerAvrg(int _n=30)
    {
        n=_n;
        times.reserve(n);
    }
    inline void start(){
        begin= std::chrono::high_resolution_clock::now();

    }

    inline void stop(){
        end= std::chrono::high_resolution_clock::now();

        double duration=double(std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())*1e-6;
        if ( times.size()<n) times.push_back(duration);
        else{
            times[curr]=duration;
            curr++;
            if (curr>=times.size()) curr=0;
        }
    }

    void reset(){
        times.clear();
        curr=0;
    }
//returns time in seconds
   inline double getAvrg(){
        double sum=0;
        for(auto t:times) sum+=t;
        return sum/double(times.size());
    }
};

}


#endif
