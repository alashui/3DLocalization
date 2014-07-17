/**
*
*	@File Matching.cpp
* 	@Author Alex and John Summer 2014
* 	@Info Matches particles and robot state!
*
**/
#include "Matching.h"

using namespace cv;
using namespace std;

namespace MCL
{
    float GetSimilarity(Mat& mat1, Mat& mat2);
    float CompareDescriptors(Mat& desc1, Mat& desc2);

	float CompareAndWeigh(Particle * p, RobotState R, vector<float> comboweights)
	{
		Characterizer c1 = masterMap[p->GetPerspective()];
		Characterizer c2 = R.GetCharacterizer();

		float sim = 0;

        // a combination of similarity tests:
        
        if(comboweights.size() != 4)
            ErrorIO("CompareAndWeight : Length of comboweight vector must be 4\n");

        if(!(abs(comboweights[0]) < .005))
            sim += (int) CompareDescriptors(c1.surfs, c2.surfs) *comboweights[0];
        if(!(abs(comboweights[1]) < .005))
            sim += (int) CompareDescriptors(c1.sifts, c2.sifts)*comboweights[1];
        if(!(abs(comboweights[2]) < .005))
            sim += GetSimilarity(c1.gs, c2.gs)*comboweights[2];
        if(!(abs(comboweights[3]) < .005))
            sim += GetSimilarity(c1.bw, c2.bw)*comboweights[3];

        // cout
        // << "SURFS: " << 10 * (int) CompareDescriptors(c1.surfs, c2.surfs)
        // << "\tSIFTS: " << (int) CompareDescriptors(c1.sifts, c2.sifts) / 3
        // << "\tPXSUM: " << GetSimilarity(c1.pixSum, c2.pixSum)
        // << "\tABOVE: " << GetSimilarity(c1.bw, c2.bw) / 2
        // << "\tTOTAL: " << sim
        // << endl;

        p->SetWeight(sim);
        return sim;
	}


    // given two images of different size, return a similarity score
    float SimilarityOfDifferentSizedImages(Mat& mat1, Mat& mat2)
    {
    // TODO
        return 10000.0;
    }

    // given two sets of keypoints and descriptors, return a similarity score
    float CompareDescriptors(Mat& desc1, Mat& desc2)
    {
        // First match descriptors
        FlannBasedMatcher matcher;
        vector<DMatch> matches;

        double total = 0.0;
        double count = 0.0;

        vector<vector<DMatch> > vecmatches;
        float ratio = 0.75;

        matcher.knnMatch(desc1, desc2, vecmatches, 2);
        for (int i = 0; i < vecmatches.size(); i++)
            if (vecmatches[i][0].distance < ratio * vecmatches[i][1].distance)
                matches.push_back(vecmatches[i][0]);

        for( int i = 0; i < matches.size(); i++ )
        {
            total += matches[i].distance;
            count += 1.0;
        }

        if (count < 2)
            return 1000.0;
        // cout << total / count << " ";
        return 1/count; // + count / 2.0;
    }

    // Elementwise disance of two images.
    float ElementWiseDistance (Mat& mat1, Mat& mat2)
    {
        if (mat1.size() != mat2.size())
        {
            stringstream ss;
            ss << "Error in Matching.h -> ElementWiseDistance! Matrices are not the same size!";
            ErrorIO(ss.str());
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
    float GetSimilarity( Mat& mat1, Mat& mat2)
    {
        if (mat1.size() != mat2.size())
            return SimilarityOfDifferentSizedImages(mat1, mat2);

        float sim = ElementWiseDistance(mat1, mat2);
        // sim += norm(mat1, mat2);
        // ^ has issues with Mat types

        return sim;
    }
}