// Characterizer.h

#include "opencv2/core/core.hpp"
#include <cstdlib>

using namespace cv;
using namespace std;

namespace MCL {

    struct Characterizer {
        Mat image;
        Mat bw;
        Mat gs;
        Mat surfs;
        Mat sifts;
    };

}