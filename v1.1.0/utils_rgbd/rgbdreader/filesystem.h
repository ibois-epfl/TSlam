#ifndef FILESYSTEM_H
#define FILESYSTEM_H

#endif // FILESYSTEM_H

#if defined(WIN32) || defined(WIN64)
#include "win_dirent.h"
#else
#include <dirent.h>
#endif

#include <set>
#include <string>

namespace filesystem {

    std::set<std::string> get_dirs_list(std::string path);
    std::set<std::string> get_files_list(std::string path);

}
