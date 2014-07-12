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
        Perspective() : x(0), y(0), z(0), dx(0), dy(0), dz(0) { }
    };

    class ComparePerspectives { // simple comparison function
       public:
          bool operator()(const Perspective& a,const Perspective& b) const { return (a.x- b.x)>0; } // returns x>y
    };
}