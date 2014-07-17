cmake_minimum_required (VERSION 2.8)
project (OBJ-Parser)


#########################################################
# FIND OPENGL
#########################################################
find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED )

include_directories(${OpenGL_INCLUDE_DIRS})

#########################################################
# Include Files
#########################################################
add_executable(IOTest ../../../IO/ProgramIO.cpp ../main.cpp ../../../Robot/RobotIO.cpp)

########################################################
# Linking & stuff
#########################################################
target_link_libraries(IOTest GL GLU IL GLEW glut X11 pthread opencv_imgproc opencv_highgui opencv_core ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${QT_LIBRARIES} /opt/ros/hydro/lib/libcv_bridge.so /usr/lib/libopencv_core.so.2.3.1 )

