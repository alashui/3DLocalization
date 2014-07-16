/** @File    - Characterizer.h
*   @Author  - Alex Rich and John Allard, Summer 2014.
*   @Purpose - This struct is defined to conveniently hold all of the data that characterizes a certain perspective in the environment.
*              This includes an image from that perspective, a b/w image from that persp, a greyscale image, SURF descriptors, and SIFT
*              descriptors. Every perspective in our environment is associated with a characterizer via a std::map. 
**/
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