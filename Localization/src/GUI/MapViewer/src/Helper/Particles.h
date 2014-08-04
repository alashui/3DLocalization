#ifndef PARTICLES_H_
#define PARTICLES_H_

#include <IL/il.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>

 struct MyParticle
    {
#define MEMBER_OFFSET(s,m) ((char *)NULL + (offsetof(s,m)))
#define BUFFER_OFFSET(i) ((char *)NULL + (i))
#define VERTICES 144

        static const int PARTICLE = 0;
        static const int BESTGUESS = 1;
        static const int AVGGUESS = 2;
        float maxweight;
        float minweight;

      GLuint vertexbuffer;
      GLuint colorbuffer;
      GLuint normalbuffer;

      GLfloat g_vertex_buffer_data[VERTICES];
      GLfloat g_color_buffer_data[VERTICES];
      GLfloat g_normal_buffer_data[VERTICES];

      MyParticle(float x, float y, float z, float dx, float dy, float dz, float weight, float count, int state)
      {
        maxweight = 45;
        minweight = 0;
        srand(time(0));

        x *= 2; y *= 2;
        // // ================================= //
        // // ========== VERTICES =========== //
        // // ================================= //
        const GLfloat hold[] = {
                        1, 1, 1, -1, 1, 1, -1,-1, 1, // v0-v1-v2 (front)
                       -1,-1, 1, 1,-1, 1, 1, 1, 1, // v2-v3-v0
                        1, 1, 1, 1,-1, 1, 1,-1,-1, // v0-v3-v4 (right)
                        1,-1,-1, 1, 1,-1, 1, 1, 1, // v4-v5-v0
                        1, 1, 1, 1, 1,-1, -1, 1,-1, // v0-v5-v6 (top)
                       -1, 1,-1, -1, 1, 1, 1, 1, 1, // v6-v1-v0
                       -1, 1, 1, -1, 1,-1, -1,-1,-1, // v1-v6-v7 (left)
                       -1,-1,-1, -1,-1, 1, -1, 1, 1, // v7-v2-v1
                       -1,-1,-1, 1,-1,-1, 1,-1, 1, // v7-v4-v3 (bottom)
                        1,-1, 1, -1,-1, 1, -1,-1,-1, // v3-v2-v7
                        1,-1,-1, -1,-1,-1, -1, 1,-1, // v4-v7-v6 (back)
                       -1, 1,-1, 1, 1,-1, 1,-1,-1, // v6-v5-v4
                       0.5, 0.5, -0.2, -0.5, -0.5, -0.2, 5*dx, 5*dy, 0.0, // bottom
                       0.5, 0.5, 0.2, -0.5, -0.5, 0.2, 5*dx, 5*dy, 0.0, // top
                      -0.5, -0.5, -0.2, -0.5, -0.5, 0.2, 5*dx, 5*dy, 0.0, // left
                       0.5, 0.5, -0.2, 0.5, 0.5, 0.2, 5*dx, 5*dy, 0.0 // right
                     };

          if(state == BESTGUESS)
          {
            for(int i = 0, j = 0; i < VERTICES; i+=3)
            {
              g_vertex_buffer_data[j++] = (hold[j]/10.0)+x;
              g_vertex_buffer_data[j++] = (hold[j]/10.0)+y;
              g_vertex_buffer_data[j++] = (hold[j]/3)+1;
            }
          }
          else if(state == AVGGUESS)
          {
            for(int i = 0, j = 0; i < VERTICES; i+=3)
            {
              g_vertex_buffer_data[j++] = (hold[j]/10.0)+x;
              g_vertex_buffer_data[j++] = (hold[j]/10.0)+y;
              g_vertex_buffer_data[j++] = (hold[j]/3)+1;
            }
          }
          else
          {
            for(int i = 0, j = 0; i < VERTICES; i+=3)
            {
              g_vertex_buffer_data[j++] = (hold[j]/16.0)+x;
              g_vertex_buffer_data[j++] = (hold[j]/16.0)+y;
              g_vertex_buffer_data[j++] = (hold[j]/60.0)+ z + count*0.03;
            }
          }

          glGenBuffers(1, &vertexbuffer);
          glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
          glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);


          // ================================== //
          // ======== COLOR DATA!!! ========== //
          // ================================== //
          if(weight < minweight)
            weight = 1;
          if(weight > maxweight)
            weight = 39.5;

          if(state == BESTGUESS)
          {
            for(int i = 0, j = 0; i < VERTICES; i+=3)
            {
              g_color_buffer_data[j++] = 0;
              g_color_buffer_data[j++] = 1;
              g_color_buffer_data[j++] = 0;
            }
          }
          else if(state == AVGGUESS)
          {
            for(int i = 0, j = 0; i < VERTICES; i+=3)
            {
              g_color_buffer_data[j++] = 0.9;
              g_color_buffer_data[j++] = 0.9;
              g_color_buffer_data[j++] = 0.9;
            }
          }
          else
          {
              for(int i = 0, j = 0; i < VERTICES; i+=3)
              {
                g_color_buffer_data[j++] = weight/maxweight;
                g_color_buffer_data[j++] = 0;
                g_color_buffer_data[j++] =(1-weight/maxweight < 0)? 0 : (1-weight/maxweight);
              }
          }
          glGenBuffers(1, &colorbuffer);
          glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
          glBufferData(GL_ARRAY_BUFFER, sizeof(g_color_buffer_data), g_color_buffer_data, GL_STATIC_DRAW);


          // ==================================== //
          // ======== NORMALS DATA!!! ========== //
          // ==================================== //
          // normal array
          GLfloat normals1[] = {
                0, 0, 1, 0, 0, 1, 0, 0, 1, // v0-v1-v2 (front)
                0, 0, 1, 0, 0, 1, 0, 0, 1, // v2-v3-v0
                1, 0, 0, 1, 0, 0, 1, 0, 0, // v0-v3-v4 (right)
                1, 0, 0, 1, 0, 0, 1, 0, 0, // v4-v5-v0
                0, 1, 0, 0, 1, 0, 0, 1, 0, // v0-v5-v6 (top)
                0, 1, 0, 0, 1, 0, 0, 1, 0, // v6-v1-v0
               -1, 0, 0, -1, 0, 0, -1, 0, 0, // v1-v6-v7 (left)
               -1, 0, 0, -1, 0, 0, -1, 0, 0, // v7-v2-v1
                0,-1, 0, 0,-1, 0, 0,-1, 0, // v7-v4-v3 (bottom)
                0,-1, 0, 0,-1, 0, 0,-1, 0, // v3-v2-v7
                0, 0,-1, 0, 0,-1, 0, 0,-1, // v4-v7-v6 (back)
                0, 0,-1, 0, 0,-1, 0, 0,-1, // v6-v5-v4
                0, 0,-1, 0, 0,-1, 0, 0,-1,
                0, 0, 1, 0, 0, 1, 0, 0, 1,
               -1, 0, 0, -1, 0, 0, -1, 0, 0,
                1, 0, 0, 1, 0, 0, 1, 0, 0
              };


          for(int i = 0; i < VERTICES; i++)
            g_normal_buffer_data[i] = normals1[i];

          glGenBuffers(1, &normalbuffer);
          glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
          glBufferData(GL_ARRAY_BUFFER, sizeof(g_normal_buffer_data), g_normal_buffer_data, GL_STATIC_DRAW);

      }

        void draw()
        {
    
    // enble and specify pointers to vertex arrays
    glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    // glEnableClientState(GL_VERTEX_ARRAY);

                glEnableVertexAttribArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
            glVertexAttribPointer(
               0, // attribute 0. No particular reason for 0, but must match the layout in the shader.
               3, // size
               GL_FLOAT, // type
               GL_FALSE, // normalized?
               0, // stride
               (void*)0 // array buffer offset
            );

            // glNormalPointer(GL_FLOAT, 0, &(g_normal_buffer_data[0]));

          glEnableVertexAttribArray(1);
          glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
          glVertexAttribPointer(
              1, // attribute. No particular reason for 1, but must match the layout in the shader.
              3, // size
              GL_FLOAT, // type
              GL_FALSE, // normalized?
              0, // stride
              (void*)0 // array buffer offset
          );

             glEnableVertexAttribArray(2);
          glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
          glVertexAttribPointer(
              2, // attribute. No particular reason for 1, but must match the layout in the shader.
              3, // size
              GL_FLOAT, // type
              GL_FALSE, // normalized?
              0, // stride
              (void*)0 // array buffer offset
          );


    glDrawArrays(GL_TRIANGLES, 0, VERTICES);


    // glDisableClientState(GL_VERTEX_ARRAY);  // disable vertex arrays
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);

        }

        void destroy()
        {

        }

    };


#endif