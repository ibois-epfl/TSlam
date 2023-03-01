#ifndef RGBDREADERFACTORY_H
#define RGBDREADERFACTORY_H

#include "rgbdreader.h"
class RGBDReaderFactory
{
public:
    RGBDReaderFactory();
    static std::shared_ptr<RGBDReader> getReader(std::string rt,std::string dir_path);
};

#endif // RGBDREADERFACTORY_H
