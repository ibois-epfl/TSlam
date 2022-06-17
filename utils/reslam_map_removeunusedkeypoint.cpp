#include "reslam.h"
int main(int argc,char **argv){

    try {
        if(argc!=3){
            cerr<<"InMap OutMap"<<endl;
        }
        reslam::Map map;
        map.readFromFile(argv[1]);
        map.removeUnUsedKeyPoints();
        map.saveToFile(argv[2]);

    } catch (std::exception &ex) {
        cout<<ex.what()<<endl;
    }
}
