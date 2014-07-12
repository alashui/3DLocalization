// Characterizer.h

#include "opencv2/core/core.hpp"
#include <cstdlib>

using namespace cv;
using namespace std;

struct characterizer {
    Mat image;
    Mat bw;
    Mat gs;
    Mat surfs;
    Mat sifts;
}