#ifndef _TSLAM_OSADAPTER_
#define _TSLAM_OSADAPTER_

#ifdef WIN32
  #define __PRETTY_FUNCTION__  __FUNCDNAME__
  #define __func__ __FUNCTION__

#elif __linux__

#endif
#endif
