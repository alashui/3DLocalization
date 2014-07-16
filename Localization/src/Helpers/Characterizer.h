// Characterizer.h

#ifndef MCL_CHARACTERIZER_H_
#define MCL_CHARACTERIZER_H_

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

#endif