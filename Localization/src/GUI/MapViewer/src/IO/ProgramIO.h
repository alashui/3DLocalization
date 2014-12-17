#ifndef PROGRAMIO_H_
#define PROGRAMIO_H_

#include "../Helper/HelperStructures.h"
#include "../Helper/MathHelp.h"
#include "../View/View.h"
#include "../Rendering/Render.h"
#include "../../../../Particle/Particle.h"
#include <boost/algorithm/string.hpp>

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <fstream>
#include <cmath>


// Create an instance of the Importer class
extern Assimp::Importer importer;
extern int startX, startY, tracking;
extern float alpha;
extern float beta;
extern float r;

using namespace MCL;

namespace IO
{
    // This function parses the input file to gather all of the necessary information from the user.
    // The input file follows this format
    //-------------------------------------------------------------
    // <path to the folder that contains the .obj, .mtl, and texture images.>
    // <name of the .obj file, not the path, just the name xxx.obj>
    // <path to the directory that you want the images saved into>
    // <type of extension you want the images to be saved under>
    // <List of all of the perspective from within the map you want images rendered from. These are in the form :
    // x y z lookx looky lookz
    // where x, y, z are the coordinates of the camera and lookx, looky, lookz is the vector the camera is looking along.
    // This list can be as long as possible, just make usre each line has those 6 values listed above >
    bool parseInputFile(const char *);

    void Idle(void);

    bool generatePoints(char *);

    // round a floating point number to two decimal places
    float round(float x);
    

    // Finds and loads the .obj file that contains the 3D model data.
    // Pass it the full file name of the .obj file relative to the build folder!
    bool Import3DFromFile();

    // Loads the texture information from the .mtl files and the texture images and applies it to our polygon mesh
    // The paths to the texture files are already loaded in the parseInputFile function
    int LoadGLTextures();

    // A simple helper function to display error and success data to the user.
    void printProgramInfoLog(GLuint obj);

    // a Callback function that the program sends a message to when the user tries to resize the window. 
    // Computes the ratio fo the width and heigh of the screen and adjusts the view matrix accordingly.
    void changeSize(int w, int h);

    // This function is called to have the viewer jump to either the next location in the list or to the location specified by the parameter.
    // Pass in a negative number to jump to the next location sequentially, if you pass in a positive number it will attempt to jump the location
    // at that index in the perspectiveList vector.
    void nextLocation(int);

    void processSpecialKey(int key, int xx, int yy);

    // process keyboard input
    void processKeys(unsigned char key, int xx, int yy);

    // process mouse button input
    void processMouseButtons(int button, int state, int xx, int yy);

    std::vector<Particle> GetParticleList();

    void processMouseMotion(int xx, int yy);

    void mouseWheel(int wheel, int direction, int x, int y);

    #define printOpenGLError() printOglError(__FILE__, __LINE__)
    int printOglError(char *file, int line);

} // end namespace IO

#endif
