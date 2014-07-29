#ifndef HELPERSTRUCTURES_H_
#define HELPERSTRUCTURES_H_

#include <IL/il.h>
#include <GL/glew.h>
#include <GL/freeglut.h>

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

    struct MyParticle
    {
        GLuint VertexVBOID;
        GLuint IndexVBOID;
        Helper::MyVertex pvertex[5];
        float vals[15];
        ushort pindices[18];
        float diff;

        MyParticle(float x, float y, float z)
        {
            diff = 0.3;
            pvertex[0].x = x - diff;
            pvertex[0].y = y + diff;
            pvertex[0].z = z;
            pvertex[0].nx = 0.0;
            pvertex[0].ny = 0.0;
            pvertex[0].nz = 1.0;
            pvertex[0].s0 = 0.0;
            pvertex[0].t0 = 0.0;
            //VERTEX 1
            pvertex[1].x = x+diff;
            pvertex[1].y = y+diff;
            pvertex[1].z = z;
            pvertex[1].nx = 0.0;
            pvertex[1].ny = 0.0;
            pvertex[1].nz = 1.0;
            pvertex[1].s0 = 1.0;
            pvertex[1].t0 = 0.0;
            //VERTEX 2
            pvertex[2].x = x-diff;
            pvertex[2].y = y-diff;
            pvertex[2].z = z;
            pvertex[2].nx = 0.0;
            pvertex[2].ny = 0.0;
            pvertex[2].nz = 1.0;
            pvertex[2].s0 = 0.0;
            pvertex[2].t0 = 1.0;

            pvertex[3].x = x+diff;
            pvertex[3].y = y-diff;
            pvertex[3].z = z;
            pvertex[3].nx = 0.0;
            pvertex[3].ny = 0.0;
            pvertex[3].nz = 1.0;
            pvertex[3].s0 = 0.0;
            pvertex[3].t0 = 1.0;

            pvertex[4].x = x;
            pvertex[4].y = y;
            pvertex[4].z = z+ diff;
            pvertex[4].nx = 0.0;
            pvertex[4].ny = 0.0;
            pvertex[4].nz = 1.0;
            pvertex[4].s0 = 0.0;
            pvertex[4].t0 = 1.0;
         
         // static int x1 = 0;
         // if(x1 == 0)
         // {
            for(int i = 0, j = 0; i < 5; i++)
            {
              vals[j++] = pvertex[i].x;
              vals[j++] = pvertex[i].y;
              vals[j++] = pvertex[i].z;
            }

          glGenBuffers(1, &VertexVBOID);
          glBindBuffer(GL_ARRAY_BUFFER, VertexVBOID);
          glBufferData(GL_ARRAY_BUFFER, sizeof(float)*3*5, vals, GL_DYNAMIC_DRAW);
        
          pindices[0] = 0;
          pindices[1] = 1;
          pindices[2] = 2;

          pindices[3] = 2;
          pindices[4] = 1;
          pindices[5] = 3;

          pindices[6] = 0;
          pindices[7] = 1;
          pindices[8] = 4;

          pindices[9] = 0;
          pindices[10] = 2;
          pindices[11] = 4;

          pindices[12] = 1;
          pindices[13] = 3;
          pindices[14] = 4;

          pindices[15] = 2;
          pindices[16] = 3;
          pindices[17] = 4;
         
         // if(x1 == 0)
         // {
          glGenBuffers(1, &IndexVBOID);
          glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexVBOID);
          glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(ushort)*12, pindices, GL_DYNAMIC_DRAW);

            #define BUFFER_OFFSET(i) ((char *)NULL + (i))

          glBindBuffer(GL_ARRAY_BUFFER, VertexVBOID);
          glEnableVertexAttribArray(0);    //We like submitting vertices on stream 0 for no special reason
          glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Helper::MyVertex), BUFFER_OFFSET(0));   //The starting point of the VBO, for the vertices
          glEnableVertexAttribArray(1);    //We like submitting normals on stream 1 for no special reason
          glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Helper::MyVertex), BUFFER_OFFSET(20));     //The starting point of normals, 12 bytes away
          glEnableVertexAttribArray(2);    //We like submitting texcoords on stream 2 for no special reason
          glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Helper::MyVertex), BUFFER_OFFSET(92));   //The starting point of texcoords, 24 bytes away

          // //VERTEX 0
          // pvertex[0].x = 0.0+x;
          // pvertex[0].y = 0.0+y;
          // pvertex[0].z = 0.0+z;
          // pvertex[0].nx = 0.0;
          // pvertex[0].ny = 0.0;
          // pvertex[0].nz = 1.0;
          // pvertex[0].s0 = 0.0;
          // pvertex[0].t0 = 0.0;
          // //VERTEX 1
          // pvertex[1].x = 1.0+x;
          // pvertex[1].y = 0.0+y;
          // pvertex[1].z = 0.0+z;
          // pvertex[1].nx = 0.0;
          // pvertex[1].ny = 0.0;
          // pvertex[1].nz = 1.0;
          // pvertex[1].s0 = 1.0;
          // pvertex[1].t0 = 0.0;
          // //VERTEX 2
          // pvertex[2].x = 0.0+x;
          // pvertex[2].y = 1.0+y;
          // pvertex[2].z = 0.0+z;
          // pvertex[2].nx = 0.0;
          // pvertex[2].ny = 0.0;
          // pvertex[2].nz = 1.0;
          // pvertex[2].s0 = 0.0;
          // pvertex[2].t0 = 1.0;
         
          // glGenBuffers(1, &VertexVBOID);
          // glBindBuffer(GL_ARRAY_BUFFER, VertexVBOID);
          // glBufferData(GL_ARRAY_BUFFER, sizeof(MyVertex)*3, &pvertex[0].x, GL_STATIC_DRAW);
         
          // // ushort pindices[3];
          // pindices[0] = 0;
          // pindices[1] = 1;
          // pindices[2] = 2;
         
          // glGenBuffers(1, &IndexVBOID);
          // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexVBOID);
          // glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(ushort)*3, pindices, GL_STATIC_DRAW);
         
          // //Define this somewhere in your header file
          // // define BUFFER_OFFSET(i) ((char *)NULL + (i))
         
          // glBindBuffer(GL_ARRAY_BUFFER, VertexVBOID);
          // glEnableVertexAttribArray(0);    //We like submitting vertices on stream 0 for no special reason
          // glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex), BUFFER_OFFSET(0));   //The starting point of the VBO, for the vertices
          // glEnableVertexAttribArray(1);    //We like submitting normals on stream 1 for no special reason
          // glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex), BUFFER_OFFSET(12));     //The starting point of normals, 12 bytes away
          // glEnableVertexAttribArray(2);    //We like submitting texcoords on stream 2 for no special reason
          // glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(MyVertex), BUFFER_OFFSET(24));   //The starting point of texcoords, 24 bytes awa

        }



        void draw()
        {
          glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexVBOID);
          //To render, we can either use glDrawElements or glDrawRangeElements
          //The is the number of indices. 3 indices needed to make a single triangle
          glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_SHORT, BUFFER_OFFSET(0));   //The starting point of the IB

      }

    };

    
}

#endif