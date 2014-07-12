// Perspective.h
#include <cstdlib>

struct perspective
{
    float x;
    float y;
    float z;
    float thetax;
    float thetay;
    float thetaz;

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