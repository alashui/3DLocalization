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

#include "../Helpers/Globals/Globals.h"
#include "../Helpers/Perspective.h"
#include "../Helpers/Characterizer.h"

using namespace cv;
using namespace std;

namespace MCL
{
    float getSimilarity(Mat& mat1, Mat& mat2);
    float compareDescriptors(Mat& desc1, Mat& desc2);

	float compareAndWeigh(Particle p, RobotState R)
	{
		Characterizer c1 = masterMap[p.getPerspective()];
		Characterizer c2 = R.getCharacterizer();

		float sim = 0;

        // a combination of similarity tests:
        
        // sim += (int) compareDescriptors(if1.surfs, if2.surfs) * 10;
        // sim += (int) compareDescriptors(if1.sifts, if2.sifts) ;/// 3;
        sim += getSimilarity(c1.gs, c2.gs);
        // sim += getSimilarity(if1.bw, if2.bw);

        // cout
        // << "SURFS: " << 10 * (int) compareDescriptors(if1.surfs, if2.surfs)
        // << "\tSIFTS: " << (int) compareDescriptors(if1.sifts, if2.sifts) / 3
        // << "\tPXSUM: " << getSimilarity(if1.pixSum, if2.pixSum)
        // << "\tABOVE: " << getSimilarity(if1.bw, if2.bw) / 2
        // << "\tTOTAL: " << sim
        // << endl;

        return sim;
	}


    // given two images of different size, return a similarity score
    float similarityOfDifferentSizedImages(Mat& mat1, Mat& mat2)
    {
    // TODO
        return 10000.0;
    }

    // given two sets of keypoints and descriptors, return a similarity score
    float compareDescriptors(Mat& desc1, Mat& desc2)
    {
        // First match descriptors
        FlannBasedMatcher matcher;
        vector<DMatch> matches;
        matcher.match( desc1, desc2, matches );

        double max_dist = 0; double min_dist = 100;

        for( int i = 0; i < matches.size(); i++ )
        { 
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }

        double total = 0.0;
        double count = 0.0;

        // Only look at the good ones
        for( int i = 0; i < matches.size(); i++ )
        {
            if( matches[i].distance <= max(2*min_dist, 0.02) )
            { 
                total += matches[i].distance;
                count += 1.0;
            }
        }

        if (count < 2)
            return 1000.0;

        return total / count + count / 2.0;
    }

    // Elementwise disance of two images.
    float elementWiseDistance (Mat& mat1, Mat& mat2)
    {
        if (mat1.size() != mat2.size())
        {
            cout << "Error in Matching.h -> elementWiseDistance! Matrices are not the same size!";
            return -1;
        }

        float difference = 0.0;

        for (int r = 0; r < mat1.rows; r++)
            for (int c = 0; c < mat1.cols; c++)
                difference += abs((int) (mat1.at<Vec<uchar, 1> >(r,c))[0] - (int) (mat2.at<Vec<uchar, 1> >(4*r,4*c))[0]);
            // ^ Total sketch with that (4*r, 4*c), but hey--if it works...

        // return mean difference
        return difference / (float) (mat1.rows * mat1.cols);
    }

    // Given two images, return a similarity score.
    float getSimilarity( Mat& mat1, Mat& mat2)
    {
        if (mat1.size() != mat2.size())
            return similarityOfDifferentSizedImages(mat1, mat2);

        float sim = elementWiseDistance(mat1, mat2);
        // sim += norm(mat1, mat2);
        // ^ has issues with Mat types

        return sim;
    }
}

#endif