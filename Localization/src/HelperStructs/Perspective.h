// Perspective.h
#include <cstdlib>
#include <vector>

namespace MCL
{
    struct Perspective
    {
        float x;
        float y;
        float z;
        float dx;
        float dy;
        float dz;

        Perspective(std::vector<float> v) : x(v[0]), y(v[1]), z(v[2]), dx(v[3]), dy(v[4]), dz(v[5]) { }
        Perspective(Perspective p) : x(p.x)_, y(p.y), z(p.z), dx(p.dx), dy(p.dy), dz(p.dz) { }
        Perspective() : x(0), y(0), z(0), dx(1), dy(0), dz(0) { }

        std::vector<float> ToVector() 
        {
            std::vector<float> p(6); p.push_back(x); p.push_back(y); p.push_back(z); p.push_back(dx); p.push_back(dy); p.push_back(dz);
            return p;
        }
    };

    class ComparePerspectives { // simple comparison function
       public:
          bool operator()(const Perspective& a,const Perspective& b) const { return (a.x- b.x)>0; } // returns x>y
    };
}