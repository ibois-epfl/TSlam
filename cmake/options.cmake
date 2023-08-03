option(TSLAM_USE_OWN_CGAL "Set to OFF to use a local CGAL version"         ON)
option(TSLAM_USE_OWN_CILANTRO "Set to OFF to use a local Cilantro version" ON)
option(TSLAM_USE_OWN_EIGEN3 "Set to OFF to use a standard eigen3 version"  ON)

option(TSLAM_USE_AVX "Use AVX instructions ON/OFF " ON)

option(BUILD_4_API "Set to ON to build TSlam just with API" OFF)

option(TSLAM_USE_TIMERS "SET ON/OFF for see time of execution" OFF)
option(TSLAM_XFEATURES2D "SET ON/OFF" OFF)

option(TSLAM_BUILD_UTILS "Set to OFF to not build utils" ON)
option(TSLAM_BUILD_TESTS "Set to OFF to not build tests" OFF)
option(TSLAM_BUILD_DEBUGTESTS "Set to OFF to not build tests" OFF)
option(TSLAM_BUILD_DEBPACKAGE "Set to ON to build deb package" OFF)
