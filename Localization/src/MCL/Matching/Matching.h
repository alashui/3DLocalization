/**
*
*	@File Matching.h
* 	@Author Alex and John Summer 2014
* 	@Info Matches particles and robot state!
*
**/
#ifndef MCL_MATCHING_H_
#define MCL_MATCHING_H_

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"

#include "../../Helpers/Globals/Globals.h"
#include "../../Helpers/Perspective.h"
#include "../../Helpers/Characterizer.h"
#include "../../IO/ProgramIO.h"
#include "../../Particle/Particle.h"
#include "../../Robot/RobotState.h"

using namespace cv;
using namespace std;

namespace MCL
{
	float CompareAndWeigh(Particle*, RobotState, vector<float>);

    // given two images of different size, return a similarity score
    float SimilarityOfDifferentSizedImages(Mat&, Mat&);

    // given two sets of keypoints and descriptors, return a similarity score
    float CompareDescriptors(Mat&, Mat&);

    // Elementwise disance of two images.
    float ElementWiseDistance (Mat&, Mat&);

    // Given two images, return a similarity score.
    float GetSimilarity( Mat&, Mat&);
}

#endif