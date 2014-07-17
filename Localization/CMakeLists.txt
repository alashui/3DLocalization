cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project( Localization )
 
find_package( OpenCV REQUIRED )

set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.46.1 COMPONENTS filesystem system)

if(Boost_FOUND)
    include_directories( SYSTEM ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}) 
    add_executable( localize src/Main/Main.cpp 
    	src/Helpers/Globals/Globals.cpp
    	src/IO/ProgramIO.cpp
    	src/MCL/ActiveParticles/ActiveParticles.cpp
    	src/MCL/Control/Controller.cpp
    	src/Particle/Particle.cpp
    	src/Robot/RobotIO.cpp
    	src/Robot/RobotState.cpp)
    target_link_libraries(localize ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
endif()
