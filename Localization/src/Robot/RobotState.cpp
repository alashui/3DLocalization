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
    void RobotState::GenerateCharacterizer(Mat& img)
    {
        int divs = 50;

        vector<KeyPoint> kps;
        Mat descs;

        SurfFeatureDetector SurfDetector (3000, 6, 2, true, true);
        SurfDescriptorExtractor SurfExtractor;
        SurfDetector.detect(img, kps);
        SurfExtractor.compute(img, kps, descs);
        this->c.descs = descs;
        this->c.kps = kps;

        Mat grs = averageImage::getPixSumFromImage(img, divs);

        this->c.gs = grs;
        this->c.bw = averageImage::aboveBelow(grs);
        this->c.image = img;
    }

    Characterizer RobotState::GetCharacterizer() 
    { return this->c; }

    void RobotState::SetCharacterizer(Characterizer cnew) 
    { this->c = cnew; }

    Perspective RobotState::GetGuessPerspective() 
    { return this->pGuess; }

    void RobotState::SetGuessPerspective(Perspective newp)
    { this->pGuess = newp; }

    Perspective RobotState::GetWeightedPerspective() 
    { return this->pWeighted; }

    void RobotState::SetWeightedPerspective(Perspective newp)
    { this->pWeighted = newp; }
}