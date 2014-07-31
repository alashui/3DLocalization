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
      #define MEMBER_OFFSET(s,m) ((char *)NULL + (offsetof(s,m)))
      #define BUFFER_OFFSET(i) ((char *)NULL + (i))
      #define VERTICES 108
      GLuint vertexbuffer;
      GLuint colorbuffer;

      GLfloat g_vertex_buffer_data[108];
      GLfloat g_color_buffer_data[108];

      MyParticle(float x, float y, float z, float weight)
      {
          static const GLfloat hold[] = {
              -1.0f,-1.0f,-1.0f, // triangle 1 : begin
              -1.0f,-1.0f, 1.0f,
              -1.0f, 1.0f, 1.0f, // triangle 1 : end
              1.0f, 1.0f,-1.0f, // triangle 2 : begin
              -1.0f,-1.0f,-1.0f,
              -1.0f, 1.0f,-1.0f, // triangle 2 : end
              1.0f,-1.0f, 1.0f,
              -1.0f,-1.0f,-1.0f,
              1.0f,-1.0f,-1.0f,
              1.0f, 1.0f,-1.0f,
              1.0f,-1.0f,-1.0f,
              -1.0f,-1.0f,-1.0f,
              -1.0f,-1.0f,-1.0f,
              -1.0f, 1.0f, 1.0f,
              -1.0f, 1.0f,-1.0f,
              1.0f,-1.0f, 1.0f,
              -1.0f,-1.0f, 1.0f,
              -1.0f,-1.0f,-1.0f,
              -1.0f, 1.0f, 1.0f,
              -1.0f,-1.0f, 1.0f,
              1.0f,-1.0f, 1.0f,
              1.0f, 1.0f, 1.0f,
              1.0f,-1.0f,-1.0f,
              1.0f, 1.0f,-1.0f,
              1.0f,-1.0f,-1.0f,
              1.0f, 1.0f, 1.0f,
              1.0f,-1.0f, 1.0f,
              1.0f, 1.0f, 1.0f,
              1.0f, 1.0f,-1.0f,
              -1.0f, 1.0f,-1.0f,
              1.0f, 1.0f, 1.0f,
              -1.0f, 1.0f,-1.0f,
              -1.0f, 1.0f, 1.0f,
              1.0f, 1.0f, 1.0f,
              -1.0f, 1.0f, 1.0f,
              1.0f,-1.0f, 1.0f
          };

          for(int i = 0, j = 0; i < VERTICES/3; i++)
          {
            g_vertex_buffer_data[j++] = (hold[j]/16.0)+x;
            g_vertex_buffer_data[j++] = (hold[j]/16.0)+y;
            g_vertex_buffer_data[j++] = (hold[j]/32.0)+z;
          }
          // Generate 1 buffer, put the resulting identifier in vertexbuffer
          glGenBuffers(1, &vertexbuffer);
           
          // The following commands will talk about our 'vertexbuffer' buffer
          glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
           
          // Give our vertices to OpenGL.
          glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

          float max = 55;
          float min = 0;

          if(weight < min)
            weight = 1;
          if(weight > max)
            weight = 39.5;

          for(int i = 0, j = 0; i < VERTICES/3; i++)
          {
            g_color_buffer_data[j++] = ((weight/max)*255)/255.0;
            g_color_buffer_data[j++] = 0;
            g_color_buffer_data[j++] = (255 - 255*(weight/max))/255.0;
          }

          glGenBuffers(1, &colorbuffer);
          glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
          glBufferData(GL_ARRAY_BUFFER, sizeof(g_color_buffer_data), g_color_buffer_data, GL_STATIC_DRAW);

      }

        void draw()
        {
            glEnableVertexAttribArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
            glVertexAttribPointer(
               0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
               3,                  // size
               GL_FLOAT,           // type
               GL_FALSE,           // normalized?
               0,                  // stride
               (void*)0            // array buffer offset
            );

                      // 2nd attribute buffer : colors
          glEnableVertexAttribArray(1);
          glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
          glVertexAttribPointer(
              1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
              3,                                // size
              GL_FLOAT,                         // type
              GL_FALSE,                         // normalized?
              0,                                // stride
              (void*)0                          // array buffer offset
          );
                         
            // Draw the triangle !
            glDrawArrays(GL_TRIANGLES, 0, VERTICES); // Starting from vertex 0; 3 vertices total -> 1 triangle
             
            glDisableVertexAttribArray(0);
        }

        void destroy()
        {

        }

    };

    
}

#endif