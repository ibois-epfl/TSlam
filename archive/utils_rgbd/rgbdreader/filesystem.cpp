#include "filesystem.h"
using namespace std;
namespace filesystem {

set<string> get_dirs_list(string path){
    set<string> result;
    DIR *dir=opendir(path.c_str());

    if(dir!=NULL){
        for(dirent *entry=readdir(dir);entry!=NULL;entry=readdir(dir))
            if(entry->d_type == DT_DIR)
                result.insert(entry->d_name);
        closedir(dir);
    }

    return result;
}

set<string> get_files_list(std::string path){
    set<std::string> result;
    DIR *dir=opendir(path.c_str());

    if(dir!=NULL){
        for(dirent *entry=readdir(dir);entry!=NULL;entry=readdir(dir))
            if(entry->d_type == DT_REG)
                result.insert(entry->d_name);
        closedir(dir);
    }

    return result;
}

}


