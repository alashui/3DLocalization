#ifndef HELPERSTRUCTURES_H_
#define HELPERSTRUCTURES_H_

#include <IL/il.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>


namespace Helper
{
    // Information to render each assimp node
    struct MyMesh{

        GLuint vao;
        GLuint texIndex;
        GLuint uniformBlockIndex;
        int numFaces;
    };

    // This is for a shader uniform block
    struct MyMaterial{

        float diffuse[4];
        float ambient[4];
        float specular[4];
        float emissive[4];
        float shininess;
        int texCount;
    };


    struct VERTICES
    {
        int X;    
        int Y;    
        int Z;   
        double U;   
        double V;
    };

    struct MyVertex
      {
        float x, y, z;        //Vertex
        float nx, ny, nz;     //Normal
        float s0, t0;         //Texcoord0
      };
 
}

#endif