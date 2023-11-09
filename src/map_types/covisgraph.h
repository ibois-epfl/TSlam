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
or implied, of Rafael Muñoz Salinas or Andrea Settimi and Hong-Bin Yang.
*/

#ifndef GRAPHS_H
#define GRAPHS_H

#include <utility>
#include <iostream>
#include <cassert>
#include <map>
#include <set>
#include <cstdint>
#include <vector>
#include <mutex>
#include "tslam_exports.h"
using namespace std;

namespace tslam
{


class TSLAM_API CovisGraph
{
public:
    CovisGraph();
    CovisGraph(      CovisGraph &G);
    CovisGraph & operator=(      CovisGraph &G);
    std::set<uint32_t> getNeighbors(uint32_t idx,bool includeIdx=false,float minWeight=0)  ;
    std::set<uint32_t> getNeighborsLevel2(uint32_t idx,bool includeIdx=false,float minWeight=0)  ;
    std::vector<uint32_t> getNeighborsV(uint32_t idx,bool includeIdx=false,float minWeight=0)  ;
    std::vector<uint32_t> getNeighborsVLevel2(uint32_t idx,bool includeIdx=false)  ;
    //returns the neighbors and weights sorted if required
    std::vector<pair<uint32_t, float> > getNeighborsWeights(uint32_t idx, bool sorted)  ;

    void removeNode(uint32_t idx);

    //returns all edges without repeating, being the first the smaller idx
    std::vector<std::pair<uint32_t,uint32_t> > getAllEdges(float minWeight=-1);


    std::set<uint32_t> getNodes(void)const {return _nodes;}
    //Returns the connection between the nodes if created (if not, throws exception). If created=true, the connection is created and returns a reference to the weight
    //If created, the weight is initialized to 0
    float  getEdge(uint32_t idx1, uint32_t idx2  );
    bool  isEdge(uint32_t idx1, uint32_t idx2  );
    void createIncreaseEdge(uint32_t idx1, uint32_t idx2,float increaseValue=1);
    bool decreaseRemoveEdge(uint32_t idx1, uint32_t idx2);


    inline float getWeight(uint32_t a ,uint32_t b)const{
        assert(_mweights.count(join(a,b)));
        return _mweights.find(join(a,b))->second;
    }

    //! \brief Computes the spanning tree or essential graph
    void getEG(CovisGraph & , int minWeight=0);

    //! Number of edges
    inline uint32_t size(void){return _nodes.size();}

    // I/O
    void toStream(std::ostream &str) const;
    void fromStream(std::istream &str);

    const std::map<uint64_t,float> &getEdgeWeightMap()const{return _mweights;}

    //divides a 64bit edge into its components
    inline static pair<uint32_t,uint32_t> separe(uint64_t a_b){         uint32_t *_a_b_16=(uint32_t*)&a_b;return make_pair(_a_b_16[1],_a_b_16[0]);}

    inline bool isNode(uint32_t node){return _nodes.count(node);}
    void clear();



    std::vector<uint32_t> getNeighbors_safe(uint32_t idx,bool includeIdx=false)  ;


    //returns the shortest path between the two nodes using a Breadth-first search from the org node. If returns empty, there is no path
    //excluded_links: can be used to avoid using some of the connections
    vector<uint32_t> getShortestPath(uint32_t org, uint32_t end     );




    inline static  uint64_t join(uint32_t a ,uint32_t b){
        if( a>b)swap(a,b);
        uint64_t a_b;
        uint32_t *_a_b_16=(uint32_t*)&a_b;
        _a_b_16[0]=b;
        _a_b_16[1]=a;
        return a_b;
    }


    uint64_t getSignature()const;
private:
    void addEdge(uint32_t idx1, uint32_t idx2, float w=1.0);

    std::set<uint32_t> _nodes;
    std::map<uint32_t,std::set<uint32_t> > _mgraph;//set of nodes links for each node.
    std::map<uint64_t,float> _mweights;//map of edge-weights
    std::mutex lock_mutex;
public:
    friend ostream& operator<<(ostream& os, const CovisGraph & cvg)
    {
        for (auto e : cvg._mweights)
        {
            auto edge= cvg.separe(e.first);
            os << edge.first << "--" << edge.second << " (" << e.second << ")" ;
        }
        return os;
    }

};

}

#endif // GRAPHS_H
