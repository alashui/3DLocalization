 #ifndef HELPERSTRUCTURES_H_
#define HELPERSTRUCTURES_H_

#include <IL/il.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>

 struct MyParticle
    {
      #define MEMBER_OFFSET(s,m) ((char *)NULL + (offsetof(s,m)))
      #define BUFFER_OFFSET(i) ((char *)NULL + (i))
      #define VERTICES 144
      GLuint vertexbuffer;
      GLuint colorbuffer;
      GLuint normalbuffer;

      GLfloat g_vertex_buffer_data[VERTICES];
      GLfloat g_color_buffer_data[VERTICES];
      GLuint g_normal_buffer_data[VERTICES];

      MyParticle(float x, float y, float z, float dx, float dy, float dz, float weight, float count)
      {
        srand(time(0));
        float dxx = (rand()%100-50)/10.0;
        float dyy = (rand()%100-50)/10.0;

          const GLfloat hold[] = {
              -1.0f,-1.0f,-1.0f, // triangle 1 : begin
              -1.0f,-1.0f, 1.0f,
              -1.0f, 1.0f, 1.0f, // triangle 1 : end
              1.0f, 1.0f,-1.0f, // triangle 2 : begin
              -1.0f,-1.0f,-1.0f,
              -1.0f, 1.0f,-1.0f, // triangle 2 : end
              1.0f,-1.0f, 1.0f,  // t3 b
              -1.0f,-1.0f,-1.0f,
              1.0f,-1.0f,-1.0f,  // t3 e
              1.0f, 1.0f,-1.0f,  // t4 b
              1.0f,-1.0f,-1.0f,
              -1.0f,-1.0f,-1.0f, // t4 e
              -1.0f,-1.0f,-1.0f, // t5 b
              -1.0f, 1.0f, 1.0f, 
              -1.0f, 1.0f,-1.0f, // t5 e
              1.0f,-1.0f, 1.0f,  // t6 b
              -1.0f,-1.0f, 1.0f,
              -1.0f,-1.0f,-1.0f, // t6 e
              -1.0f, 1.0f, 1.0f, // t7 b
              -1.0f,-1.0f, 1.0f,
              1.0f,-1.0f, 1.0f,  // t7 e
              1.0f, 1.0f, 1.0f,  // t8 b
              1.0f,-1.0f,-1.0f,
              1.0f, 1.0f,-1.0f, // t8 e
              1.0f,-1.0f,-1.0f, // t9 b
              1.0f, 1.0f, 1.0f,
              1.0f,-1.0f, 1.0f, // t9 e
              1.0f, 1.0f, 1.0f, // t10 b
              1.0f, 1.0f,-1.0f,
              -1.0f, 1.0f,-1.0f,// t10 e
              1.0f, 1.0f, 1.0f, // t11 b
              -1.0f, 1.0f,-1.0f,
              -1.0f, 1.0f, 1.0f,// t11 e
              1.0f, 1.0f, 1.0f, // t12 b
              -1.0f, 1.0f, 1.0f,
              1.0f,-1.0f, 1.0f,  // t12 e
              0.5, 0.5, -0.2,    // lower arrow
              -0.5, -0.5, -0.2,
              5*dx, 5*dy, 0.0,
              0.5, 0.5, 0.2,     // upper arrow
              -0.5, -0.5, 0.2,   
              5*dx, 5*dy, 0.0,
              -0.5, -0.5, -0.2,
              -0.5, -0.5, 0.2,
              5*dx, 5*dy, 0.0,
              0.5, 0.5, -0.2,
              0.5, 0.5, 0.2,
              5*dx, 5*dy, 0.0


          };

          // std::cout << dx << " " << dy << std::endl;



          for(int i = 0, j = 0; i < VERTICES; i+=3)
          {
            g_vertex_buffer_data[j++] = (hold[j]/16.0)+x;
            g_vertex_buffer_data[j++] = (hold[j]/16.0)+y;
            g_vertex_buffer_data[j++] = (hold[j]/55.0)+z + count*0.03;
          }
          // Generate 1 buffer, put the resulting identifier in vertexbuffer
          glGenBuffers(1, &vertexbuffer);
           
          // The following commands will talk about our 'vertexbuffer' buffer
          glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
           
          // Give our vertices to OpenGL.
          glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

          float max = 60;
          float min = 0;

          if(weight < min)
            weight = 1;
          if(weight > max)
            weight = 39.5;

          if(weight >= 0 && weight <= max/3)
          {
              for(int i = 0, j = 0; i < VERTICES; i+=3)
                {
                  g_color_buffer_data[j++] = weight/(2*max);
                  g_color_buffer_data[j++] = 0;
                  g_color_buffer_data[j++] = 1 - weight/max;
                }
          }
          else if(weight > max/3 && weight < 2*max/3)
          {
              for(int i = 0, j = 0; i < VERTICES; i+=3)
              {
                g_color_buffer_data[j++] = weight/max;
                g_color_buffer_data[j++] = 0;
                g_color_buffer_data[j++] = 1 - weight/max;
              }
          }
          else
          {
              for(int i = 0, j = 0; i < VERTICES; i+=3)
              {
                g_color_buffer_data[j++] = weight/max;
                g_color_buffer_data[j++] = 0;
                g_color_buffer_data[j++] = 1 - 2*weight/max;
              }
          }

          

          glGenBuffers(1, &colorbuffer);
          glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
          glBufferData(GL_ARRAY_BUFFER, sizeof(g_color_buffer_data), g_color_buffer_data, GL_STATIC_DRAW);

            static const GLfloat holdn[] = {
            -1.0f,-1.0f,-1.0f, // triangle 1 : begin
              -1.0f,-1.0f, 1.0f,
              -1.0f, 1.0f, 1.0f, // triangle 1 : end
              1.0f, 1.0f,-1.0f, // triangle 2 : begin
              -1.0f,-1.0f,-1.0f,
              -1.0f, 1.0f,-1.0f, // triangle 2 : end
              1.0f,-1.0f, 1.0f,  // t3 b
              -1.0f,-1.0f,-1.0f,
              1.0f,-1.0f,-1.0f,  // t3 e
              1.0f, 1.0f,-1.0f,  // t4 b
              1.0f,-1.0f,-1.0f,
              -1.0f,-1.0f,-1.0f, // t4 e
              -1.0f,-1.0f,-1.0f, // t5 b
              -1.0f, 1.0f, 1.0f, 
              -1.0f, 1.0f,-1.0f, // t5 e
              1.0f,-1.0f, 1.0f,  // t6 b
              -1.0f,-1.0f, 1.0f,
              -1.0f,-1.0f,-1.0f, // t6 e
              -1.0f, 1.0f, 1.0f, // t7 b
              -1.0f,-1.0f, 1.0f,
              1.0f,-1.0f, 1.0f,  // t7 e
              1.0f, 1.0f, 1.0f,  // t8 b
              1.0f,-1.0f,-1.0f,
              1.0f, 1.0f,-1.0f, // t8 e
              1.0f,-1.0f,-1.0f, // t9 b
              1.0f, 1.0f, 1.0f,
              1.0f,-1.0f, 1.0f, // t9 e
              1.0f, 1.0f, 1.0f, // t10 b
              1.0f, 1.0f,-1.0f,
              -1.0f, 1.0f,-1.0f,// t10 e
              1.0f, 1.0f, 1.0f, // t11 b
              -1.0f, 1.0f,-1.0f,
              -1.0f, 1.0f, 1.0f,// t11 e
              1.0f, 1.0f, 1.0f, // t12 b
              -1.0f, 1.0f, 1.0f,
              1.0f,-1.0f, 1.0f,  // t12 e
              0.5, 0.5, -0.4,    // lower arrow
              -0.5, -0.5, -0.4,
              5*dx, 5*dy, 0.0,
              0.5, 0.5, 0.4,     // upper arrow
              -0.5, -0.5, 0.4,   
              5*dx, 5*dy, 0.0,
              -0.5, -0.5, -0.4,
              -0.5, -0.5, 0.4,
              5*dx, 5*dy, 0.0,
              0.5, 0.5, -0.4,
              0.5, 0.5, 0.4,
              5*dx, 5*dy, 0.0

          };

          for(int i = 0; i < VERTICES; i++)
            g_normal_buffer_data[i] = -1.0*holdn[i];

          glGenBuffers(1, &normalbuffer);
          glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
          glBufferData(GL_ARRAY_BUFFER, sizeof(g_normal_buffer_data), g_normal_buffer_data, GL_STATIC_DRAW);

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

          // 3rd attribute buffer : normals
          glEnableVertexAttribArray(2);
          glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
          glVertexAttribPointer(
              2,                                // attribute
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


    #endif