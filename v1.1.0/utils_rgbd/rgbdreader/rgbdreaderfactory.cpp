#include "rgbdreaderfactory.h"
#include "sun3dreader.h"
#ifdef OPENNI2
#include "onireader.h"
#endif
#include "nyureader.h"
#include "redwoodreader.h"
using namespace std;
RGBDReaderFactory::RGBDReaderFactory(){}

std::shared_ptr<RGBDReader> RGBDReaderFactory::getReader(string rt, string path){
    if(rt=="sun3d"){
        return shared_ptr<SUN3DReader>(new SUN3DReader(path));
    }
#ifdef OPENNI2
    if(rt=="oni")
        return shared_ptr<OniReader>(new OniReader(path));
#endif
    else if(rt=="nyu1")
        return shared_ptr<NYUReader>(new NYUReader(path,"v1"));
        
    else if(rt=="nyu" || rt=="nyu2")
        return shared_ptr<NYUReader>(new NYUReader(path,"v2"));
        
    else if(rt=="redwood")
	return shared_ptr<RedwoodReader>(new RedwoodReader(path));
    throw runtime_error("Invalid reader type: "+rt+" for RGBDReader.");
}
