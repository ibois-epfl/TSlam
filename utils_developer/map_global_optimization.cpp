#include "map.h"
#include "optimization/globaloptimizer.h"
#include <iostream>
#include "basictypes/debug.h"

using namespace std;
int main(int argc,char **argv){
    try{

        if (argc!=3)throw std::runtime_error("Usage: inmap outmap");
        reslam::debug::Debug::setLevel(10);
        auto Map=std::make_shared<reslam::Map>();
        Map->readFromFile(argv[1]);

        cout<<"NKEYFRAMES="<<Map->keyframes.size() <<endl;
        auto optimizer=reslam::GlobalOptimizer::create();
        reslam::GlobalOptimizer::ParamSet params;
        params.nIters=100;
        params.verbose=true;
        optimizer->optimize(Map,params);         
//        Map->removeBadAssociations(optimizer->getBadAssociations(),3);
        Map->saveToFile(argv[2]);

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
        return -1;
    }

}
