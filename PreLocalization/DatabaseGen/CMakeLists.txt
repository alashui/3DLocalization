cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project( createDB )
 
find_package( OpenCV REQUIRED )

include_directories( SYSTEM ${OpenCV_INCLUDE_DIRS} )

link_directories( ${PCL_LIBRARY_DIRS} )
 
add_executable( CreateDB src/CreateDB.cpp )
 
target_link_libraries ( CreateDB ${OpenCV_LIBRARIES} )