#ifndef SPHERE_H_
#define SPHERE_H_

#include <vector>
#include <cmath>
#include "MathHelp.h"
#include "HelperStructures.h"

// your framework of choice here

class SolidSphere
{
protected:
    std::vector<GLfloat> vertices;
    std::vector<GLfloat> normals;
    std::vector<GLfloat> texcoords;
    std::vector<GLushort> indices;

public:
    SolidSphere(float radius, unsigned int rings, unsigned int sectors);

    void draw(GLfloat x, GLfloat y, GLfloat z);
};


#endif