// Perspective.h

#ifndef MCL_PERSPECTIVE_H_
#define MCL_PERSPECTIVE_H_

#include <cstdlib>
#include <vector>
#include <cmath>
#include <string>

namespace MCL
{
    std::vector<int> vec3(int x, int y, int z);

    struct Perspective
    {
        float x;
        float y;
        float z;
        float dx;
        float dy;
        float dz;

        Perspective(std::vector<float> v) : x(v[0]), y(v[1]), z(v[2]), dx(v[3]), dy(v[4]), dz(v[5]) { }
        Perspective(const Perspective & p) : x(p.x), y(p.y), z(p.z), dx(p.dx), dy(p.dy), dz(p.dz) { }
        Perspective() : x(0), y(0), z(0), dx(1), dy(0), dz(0) { }

        std::vector<float> ToVector() const
        {
            std::vector<float> p(6); p.push_back(x); p.push_back(y); p.push_back(z); p.push_back(dx); p.push_back(dy); p.push_back(dz);
            return p;
        }
        
        inline bool operator==(const Perspective& a) {
            float eps = 0.00001;
            std::vector<float> v1 = this->ToVector();
            std::vector<float> v2 = a.ToVector();
            for (int i = 0; i < v1.size(); i++)
                if (abs(v1[i] - v2[i]) > eps)
                    return false;
            return true;
        }
    };

    class ComparePerspectives { // simple comparison function
       public:
          bool operator()(const Perspective& a,const Perspective& b) const { return (a.x- b.x)>0; } // returns x>y
    };

}

#endif