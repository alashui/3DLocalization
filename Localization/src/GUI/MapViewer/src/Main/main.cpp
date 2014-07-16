/**
*   @File
*   @Author - John H. Allard.
*   @Data   - July 9th, 2014.
*   @Info   - Written at Harvey Mudd College as part of the 2014 Summe REU program. 
*             This file includes the main function and some helper functions for the PerspectiveGenerator program.
*             This file doesn't contain too much, it merely serves as a starting point from which to call functions in
*             the helper files and start the main rendering loop. 
**/


#include <IL/il.h>
#include <GL/glew.h>
#include <GL/freeglut.h>

#include "assimp/Importer.hpp"  //OO version Header!
#include "assimp/postprocess.h"
#include "assimp/scene.h"

#include <exception>
#include <math.h>
#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <iostream>

//#include "../Helper/MathHelp.h"
//#include "../Helper/HelperStructures.h"
#include "../View/View.h"
#include "../Rendering/Render.h"
#include "../Shaders/ShaderFunctions.h"
#include "../IO/ProgramIO.h"


int init();

int main(int argc, char **argv) 
{

//  GLUT initialization
    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_DEPTH|GLUT_DOUBLE|GLUT_RGBA);//|GLUT_MULTISAMPLE );

    glutInitContextVersion (3, 3);
    glutInitContextFlags (GLUT_COMPATIBILITY_PROFILE );

    glutInitWindowPosition(100,100);


    // if(argc == 2)
    // {
    //     if(!IO::generatePoints(argv[1]))
    //     {
    //         std::cout << "Error Generating Images. Check Input File for Errors or report on Github. \n" << std::endl;
    //     }
    // }
    // else
    // {
    //     std::cout << "You need to pass in a file of Input value.\n" << 
    //     "See /PerspectiveGenerator/ProgramDesign/InputFileFormat.txt for help" << std::endl;
    // }

    glutInitWindowSize(width, height);
    glutCreateWindow("PerspectiveGenerator");
    
    glewInit();
    if (glewIsSupported("GL_VERSION_3_3"))
        printf("OpenGL Version 3.3 detected\n");
    else {
        printf("OpenGL 3.3 not supported\n");
        return(1);
    }
        

//  Callback Registration
    glutDisplayFunc(Render::renderScene);
    glutReshapeFunc(IO::changeSize);
    glutIdleFunc(Render::renderScene);

//  Mouse and Keyboard Callbacks
    glutKeyboardFunc(IO::processKeys);
    glutMouseFunc(IO::processMouseButtons);
    glutMotionFunc(IO::processMouseMotion);
    glutMouseWheelFunc ( IO::mouseWheel ) ;

    //  Init the app (load model and textures) and OpenGL
    if (!init())
    {
        printf("Could not Load the Model Correctly. Exiting.\n");
        return 0;
    }


    // return from main loop
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
;
    glutMainLoop();

    // cleaning up
    textureIdMap.clear();  

    Render::clearMeshes();

    // delete buffers
    glDeleteBuffers(1,&matricesUniBuffer);

    return(0);
}



int init()                   
{
    if (!IO::Import3DFromFile())
    {
        std::cout << "Failed 3d model import\n"<< std::endl; 
        return(0);
    }
    IO::LoadGLTextures();

    glGetUniformBlockIndex = (PFNGLGETUNIFORMBLOCKINDEXPROC) glutGetProcAddress("glGetUniformBlockIndex");
    glUniformBlockBinding = (PFNGLUNIFORMBLOCKBINDINGPROC) glutGetProcAddress("glUniformBlockBinding");
    glGenVertexArrays = (PFNGLGENVERTEXARRAYSPROC) glutGetProcAddress("glGenVertexArrays");
    glBindVertexArray = (PFNGLBINDVERTEXARRAYPROC)glutGetProcAddress("glBindVertexArray");
    glBindBufferRange = (PFNGLBINDBUFFERRANGEPROC) glutGetProcAddress("glBindBufferRange");
    glDeleteVertexArrays = (PFNGLDELETEVERTEXARRAYSPROC) glutGetProcAddress("glDeleteVertexArrays");


    program = Shaders::setupShaders(vertexfile, fragmentfile);

    Render::genVAOsAndUniformBuffer();

    glEnable(GL_DEPTH_TEST);        
    glClearColor(1.0f, 0.0f, 1.0f, 0.0f);


    //
    // Uniform Block
    //
    glGenBuffers(1,&matricesUniBuffer);
    glBindBuffer(GL_UNIFORM_BUFFER, matricesUniBuffer);
    glBufferData(GL_UNIFORM_BUFFER, MatricesUniBufferSize,NULL,GL_DYNAMIC_DRAW);
    glBindBufferRange(GL_UNIFORM_BUFFER, matricesUniLoc, matricesUniBuffer, 0, MatricesUniBufferSize);  
    glBindBuffer(GL_UNIFORM_BUFFER,0);

    glEnable(GL_MULTISAMPLE);


    return true;                    
}
