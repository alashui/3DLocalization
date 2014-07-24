
/**
*   @File   - Render.h
*   @Author - John H. Allard.
*   @Data   - July 9th, 2014.
*   @Info   - Written at Harvey Mudd College as part of the 2014 Summe REU program. This header file and corresponding .cpp file
*             serve to hold all of the functionality needed to properly render polygons and textures to the screen for this OpenGL program.
*             This includes setting up the frame buffer, generated the vertex array objects, pushing and popping matrixes from the matrix stack,
*             and all of the actual rendering functionality. Some other helpful functionality is also included such as functions to translate, 
*             rotate, and scale the matrix.
**/

#ifndef RENDER_H_
#define RENDER_H_

#include "../Helper/MathHelp.h"
#include "../Helper/HelperStructures.h"
#include "../View/View.h"
#include "../Helper/SolidSphere.h"
#include "../IO/ProgramIO.h"


namespace Render 
{

    // A function used for debugging purposes, not currently used in the program.
    void timer(int);

    // Pop the top matrix off of the matrix stack.
    void popMatrix();

    // Push the current matrix onto the matrix stack.
    void pushMatrix();

    // Generates the Vertex Array Object and Uniform Buffer to allow us to render all of the polygon data
    // that was loaded from the load3DModel function. Generates and sets up all of the buffers needed to display to the current screen/
    void genVAOsAndUniformBuffer();

    void recursive_render (const aiNode* nd);

    void renderScene(void);

    void clearMeshes();

    void setModelMatrix();

    // The equivalent to glTranslate applied to the model matrix
    void translate(float x, float y, float z);

    // The equivalent to glRotate applied to the model matrix
    void rotate(float angle, float x, float y, float z);

    // The equivalent to glScale applied to the model matrix
    void scale(float x, float y, float z);

}

#endif