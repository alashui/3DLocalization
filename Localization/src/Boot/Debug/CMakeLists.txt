cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project( Boot )
 
find_package( OpenCV REQUIRED )

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.46.1 COMPONENTS filesystem system)

if(Boost_FOUND)
    include_directories( SYSTEM ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}) 
    add_executable( boot main.cpp )
    target_link_libraries(boot ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
endif()
