/**
* 
*   @File RobotState.cpp
*   @Author Alex Rich and John Allard, Summer 2014
*   @Info A class to contain robot state information, continually updated.
*
**/

#include "RobotState.h"


using namespace cv;
using namespace std;

namespace MCL
{
    void generateCharacterizer(Mat& img)
    {
        int divs = 50;

        vector<KeyPoint> kps;
        Mat descs;

        SurfFeatureDetector SurfDetector (3000, 6, 2, true, true);
        SurfDescriptorExtractor SurfExtractor;
        SurfDetector.detect(img, kps);
        SurfExtractor.compute(img, kps, descs);
        this->c.surfs = descs;

        SiftFeatureDetector SiftDetector (800);
        SiftDescriptorExtractor SiftExtractor;
        SiftDetector.detect(img, kps);
        SiftExtractor.compute(img, kps, descs);
        this->c.sifts = descs;

        Mat grs = averageImage::getPixSumFromImage(img, divs);

        this->c.gs = grs;
        this->c.bw = averageImage::aboveBelow(grs);
        this->image = img;
    }

    Characterizer getCharacterizer() 
    { return this->c; }

    void setCharacterizer(Characterizer cnew) 
    { this->c = cnew; }

    Perspective getPerspective() 
    { return this->p; }

    void setPerspective(Perspective newp)
    { this->p = newp; }
}