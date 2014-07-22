cmake_minimum_required (VERSION 2.8)
project (OBJ-Parser)


#########################################################
# FIND OPENGL
#########################################################
set(Boost_USE_STATIC_LIBS off)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_LIBS_RUNTIME OFF)
find_package(OpenGL REQUIRED)
find_package( OpenCV REQUIRED )
find_package(Boost 1.46.1 COMPONENTS filesystem system REQUIRED)

include_directories(${Boost_INCLUDE_DIRS} ${OpenGL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

link_directories(${OpenGL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)


SET(OBJ-Parser_SOURCES src/Main/main.cpp src/Shaders/ShaderFunctions.cpp src/View/View.cpp src/Helper/MathHelp.cpp src/Rendering/Render.cpp src/IO/ProgramIO.cpp ../../../Particle/Particle.cpp ../../../IO/ProgramIO.cpp src/Helper/SolidSphere.cpp)
SET(OBJ-Parser_HEADERS ${OBJ-Parser_HDRS} src/Shaders/ShaderFunctions.h src/View/View.h src/Helper/MathHelp.h src/Helper/HelperStructures.h src/Rendering/Render.h src/IO/ProgramIO.h)

#########################################################
# Include Files
#########################################################
add_executable(PerspectiveGenerator ${OBJ-Parser_SOURCES} ${OBJ-Parser_HEADERS_MOC} ${OBJ-Parser_FORMS_HEADERS} )

########################################################
# Linking & stuff
#########################################################
target_link_libraries(PerspectiveGenerator GL GLU IL GLEW glut X11 pthread opencv_imgproc opencv_highgui opencv_core ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${QT_LIBRARIES} /usr/local/lib/libassimp.so /opt/ros/hydro/lib/libcv_bridge.so /usr/lib/libopencv_core.so.2.3.1 )

