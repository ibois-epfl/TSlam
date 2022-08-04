OPTION(BUILD_SHARED_LIBS 	"Set to OFF to build static libraries" ON)
OPTION(USE_OWN_EIGEN3	"Set to OFF to use a standard eigen3 version" ON)
OPTION(BUILD_OWN_ARUCO	"Set to OFF to force find aruco in the system" ON)  # <---- needs to include?
OPTION(BUILD_OWN_FBOW 	"Set to OFF to force find fbow in the system" ON)
OPTION(BUILD_OWN_XFLANN "Set to OFF to force find xflann in the system" ON)
OPTION(BUILD_OWN_G2O "Set to OFF to force find g2o in the system" ON)
OPTION(BUILD_OWN_STAG "Set to OFF to force find stag in the system" ON)
OPTION(USE_AVX "SET ON/OFF " ON)

OPTION(USE_TIMERS "SET ON/OFF for see time of execution" OFF)
OPTION(XFEATURES2D "SET ON/OFF" OFF)

OPTION(BUILD_UTILS	"Set to OFF to not build utils" ON)
OPTION(BUILD_UTILS_ARRAY	"Set to OFF to not build utils" ON)
OPTION(BUILD_UTILS_DEVELOPER	"Set to OFF to not build utils" OFF)
OPTION(BUILD_TESTS	"Set to OFF to not build tests" OFF)
OPTION(BUILD_DEBUGTESTS	"Set to OFF to not build tests" OFF)
OPTION(BUILD_UTILS_RGBD	"Set to OFF to not build utils" OFF)
OPTION(BUILD_GUI	"Set to ON to build gui" OFF)
OPTION(BUILD_DEBPACKAGE	"Set to ON to build deb package" OFF)
OPTION(BUILD_UTILS_REALSENSE	"Set to OFF to not build utils" OFF)

