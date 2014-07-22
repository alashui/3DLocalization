#include "SolidSphere.h"

SolidSphere::SolidSphere(float radius, unsigned int rings, unsigned int sectors)
    {
         {
        float const R = 1./(float)(rings-1);
        float const S = 1./(float)(sectors-1);
        int r, s;

        vertices.resize(rings * sectors * 3);
        normals.resize(rings * sectors * 3);
        texcoords.resize(rings * sectors * 2);
        std::vector<GLfloat>::iterator v = vertices.begin();
        std::vector<GLfloat>::iterator n = normals.begin();
        std::vector<GLfloat>::iterator t = texcoords.begin();
        for(r = 0; r < rings; r++) for(s = 0; s < sectors; s++) {
                float const y = sin( -M_PI_2 + M_PI * r * R );
                float const x = cos(2*M_PI * s * S) * sin( M_PI * r * R );
                float const z = sin(2*M_PI * s * S) * sin( M_PI * r * R );

                *t++ = s*S;
                *t++ = r*R;

                *v++ = x * radius;
                *v++ = y * radius;
                *v++ = z * radius;

                *n++ = x;
                *n++ = y;
                *n++ = z;
        }

        indices.resize(rings * sectors * 4);
        std::vector<GLushort>::iterator i = indices.begin();
        for(r = 0; r < rings-1; r++) for(s = 0; s < sectors-1; s++) {
                *i++ = r * sectors + s;
                *i++ = r * sectors + (s+1);
                *i++ = (r+1) * sectors + (s+1);
                *i++ = (r+1) * sectors + s;
        }

        struct Helper::MyMesh aMesh;
        struct Helper::MyMaterial aMat; 
        GLuint buffer;

        glGenVertexArrays(1,&(aMesh.vao));
        glBindVertexArray(aMesh.vao);

        float* vertices2 = new float[vertices.size()];
        for(int i = 0; i < vertices.size(); i++)
        {
            vertices2[i] = vertices[i];
        }
        glGenBuffers(1, &buffer);
        glBindBuffer(GL_ARRAY_BUFFER, buffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*3*vertices.size(), mesh->mVertices, GL_STATIC_DRAW);
        glEnableVertexAttribArray(vertexLoc);
        glVertexAttribPointer(vertexLoc, 3, GL_FLOAT, 0, 0, 0);
    

        float *mNormals = (float *)malloc(sizeof(float)*normals.size());
        for (unsigned int k = 0; k < vertices.size(); ++k) {

            mNormals[k] = normals[k];
            
        }
        glGenBuffers(1, &buffer);
        glBindBuffer(GL_ARRAY_BUFFER, buffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*3*vertices.size(), mesh->mNormals, GL_STATIC_DRAW);
        glEnableVertexAttribArray(normalLoc);
        glVertexAttribPointer(normalLoc, 3, GL_FLOAT, 0, 0, 0);
        


        float *texCoords = (float *)malloc(sizeof(float)*vertices.size());
        for (unsigned int k = 0; k < vertices.size(); ++k) {

            texCoords[k] = texcoords[k];
            
        }
        glGenBuffers(1, &buffer);
        glBindBuffer(GL_ARRAY_BUFFER, buffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*vertices.size(), texCoords, GL_STATIC_DRAW);
        glEnableVertexAttribArray(texCoordLoc);
        glVertexAttribPointer(texCoordLoc, 2, GL_FLOAT, 0, 0, 0);
        

        // unbind buffers
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER,0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
    }

    void SolidSphere::draw(GLfloat x, GLfloat y, GLfloat z)
    {

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glTranslatef(x,y,z);

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);

        glVertexPointer(3, GL_FLOAT, 0, &vertices[0]);
        glNormalPointer(GL_FLOAT, 0, &normals[0]);
        glTexCoordPointer(2, GL_FLOAT, 0, &texcoords[0]);
        glDrawElements(GL_QUADS, indices.size(), GL_UNSIGNED_SHORT, &indices[0]);
        glPopMatrix();
    }