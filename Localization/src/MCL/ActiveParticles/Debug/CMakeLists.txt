cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project( ap )
 
find_package( OpenCV REQUIRED )

set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.46.1 COMPONENTS filesystem system)

if(Boost_FOUND)
    include_directories( SYSTEM ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ) 
    add_executable( ap main.cpp ../ActiveParticles.cpp ../../../Particle/Particle.cpp ../../../Helpers/Globals/Globals.cpp ../../../IO/ProgramIO.cpp ../../Control/Controller.cpp ../../../Robot/RobotState.cpp)# ../../../Robot/RobotIO.cpp)
    target_link_libraries(ap ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
endif()
