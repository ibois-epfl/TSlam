add_library(icp icp.cpp)
add_library(global_icp global_icp.cpp)
target_link_libraries(icp ${PCL_REGISTRATION_LIBRARIES})
target_link_libraries(global_icp icp)
