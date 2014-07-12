// Perspective.h
#include <cstdlib>

struct Perspective
{
    float x;
    float y;
    float z;
    float dx;
    float dy;
    float dz;

    perspective(vector<float> v)
    {
    	x = v[0];
    	y = v[1];
    	z = v[2];
    	thetax = v[3];
    	thetay = v[4];
    	thetaz = v[5];
    }
}