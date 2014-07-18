/** @File    - Perspective.h
*   @Author  - Alex Rich and John Allard, Summer 2014.
*   @Purpose - This struct is defined to conveniently hold all of the data that defines a certain perspective in the environment.
*              This includes information about the [x y z] spacial coordinated of the perspective, as well as the pdx dy dz] direction
*              that the camera is looking along from this perspective. Various functions are also included to get/set the perspective
*              with different data types.
**/
#ifndef MCL_PERSPECTIVE_H_
#define MCL_PERSPECTIVE_H_

#include <cstdlib>
#include <vector>
#include <cmath>
#include <string>
#include <iostream>
#include <sstream>

namespace MCL
{
    struct Perspective
    {
        // [x y z] spacial coordinates
        float x, y, z;

        // [dx dy dz] direction of camera orientation coordinates.
        float dx, dy, dz;

        // various constructors
        Perspective(std::vector<float> v) : x(v[0]), y(v[1]), z(v[2]), dx(v[3]), dy(v[4]), dz(v[5]) { }
        Perspective(const Perspective & p) : x(p.x), y(p.y), z(p.z), dx(p.dx), dy(p.dy), dz(p.dz) { }
        Perspective() : x(0), y(0), z(0), dx(1), dy(0), dz(0) { }
        Perspective(float lx, float ly, float lz, float ldx, float ldy, float ldz)
        {
            x = lx;
            y = ly;
            z = lz;
            dx = ldx;
            dy = ldy;
            dz = ldz;
        }

        std::vector<float> ToVector() const
        {
            std::vector<float> p; p.push_back(x); p.push_back(y); p.push_back(z); p.push_back(dx); p.push_back(dy); p.push_back(dz);
            return p;
        }

        std::string ToString() const
        {
            std::stringstream ss; 
            ss << "[" << x << ", " << y << ", " << z << "], [" << dx << ", " << dy << ", " << dz << "]";
            return ss.str();
        }

        // Used to see if two perspectives are equal within a defined epsilon (because of flotating point trunc errors.)
        inline bool operator==(const Perspective& a) const {
            std::cout << "COMPARING PERSPECTIVES: " << ToString() << " " << a.ToString() << std::endl;
            float eps = 0.001;
            if (abs(a.x - x) > eps || abs(a.y - y) > eps || abs(a.z - z) > eps)
                return false;

            float angle1 = atan2(x, y);
            float angle2 = atan2(a.x, a.y);
            if (abs(angle1 - angle2) > 0.1)
                return false;

            std::cout << "TRUE" << std::endl;
            return true;
        }
    };

    class ComparePerspectives { // simple comparison function for the std::map to use
       public:
          bool operator()(const Perspective& a,const Perspective& b) const { /*std::cout << "COMPAREPERSPECTIVES" << std::endl;*/ return (a.operator==(b)); } // returns x>y
    };

}

#endif