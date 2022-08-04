option(BUILD_SHARED_LIBS 	"Set to OFF to build static libraries" ON)
option(USE_OWN_EIGEN3	"Set to OFF to use a standard eigen3 version" ON)
option(BUILD_OWN_ARUCO	"Set to OFF to force find aruco in the system" ON)  # <---- needs to include?
option(BUILD_OWN_FBOW 	"Set to OFF to force find fbow in the system" ON)
option(BUILD_OWN_XFLANN "Set to OFF to force find xflann in the system" ON)
option(BUILD_OWN_G2O "Set to OFF to force find g2o in the system" ON)
option(BUILD_OWN_STAG "Set to OFF to force find stag in the system" ON)
option(USE_AVX "SET ON/OFF " ON)

option(USE_TIMERS "SET ON/OFF for see time of execution" OFF)
option(XFEATURES2D "SET ON/OFF" OFF)

option(BUILD_UTILS	"Set to OFF to not build utils" ON)
option(BUILD_UTILS_ARRAY	"Set to OFF to not build utils" ON)
option(BUILD_UTILS_DEVELOPER	"Set to OFF to not build utils" OFF)
option(BUILD_TESTS	"Set to OFF to not build tests" OFF)
option(BUILD_DEBUGTESTS	"Set to OFF to not build tests" OFF)
option(BUILD_UTILS_RGBD	"Set to OFF to not build utils" OFF)
option(BUILD_GUI	"Set to ON to build gui" OFF)
option(BUILD_DEBPACKAGE	"Set to ON to build deb package" OFF)
option(BUILD_UTILS_REALSENSE	"Set to OFF to not build utils" OFF)

