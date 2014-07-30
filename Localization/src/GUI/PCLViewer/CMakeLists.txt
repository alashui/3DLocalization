cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(pcl_viewer)
 
find_package(PCL 1.7 REQUIRED)
 
find_package(Boost 1.46.1 COMPONENTS filesystem system)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 

set(PCL_BUILD_TYPE Release)
 
add_executable(pcl_viewer Viewer.cpp)
 
target_link_libraries (pcl_viewer ${PCL_LIBRARIES} ${Boost_LIBRARIES})
