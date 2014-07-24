/**
*
*	@File Matching.cpp
* 	@Author Alex and John Summer 2014
* 	@Info Matches different types of feature data between a particle and the current robot state. This involved comparing the
*         SURF, B/W, and Greyscale features of both the image rendering from the particle location and the image coming from 
*         robots camera feed. Different weights are assigned to represent the quality of the matches and then they are weighed once
*         again based on how important we think each set of features are. (e.g. SURF feature are more accurate than Greyscale so we weight
*         those more). This file is really at the heart of the MCL algorithm and thus goes through constant updates.
*
**/
#include "Matching.h"

using namespace cv;
using namespace std;

namespace MCL
{
    float GetSimilarity(Mat& mat1, Mat& mat2);
    float CompareDescriptors(Mat& desc1, Mat& desc2);

	float CompareAndWeigh(Particle p, RobotState R, vector<float> comboweights)
	{
        if (!masterMap.count(p.GetPerspective()))
        {
            cout << "NOT FOUND: " << p.GetPerspective().ToString() << endl;
            return -1000.0;
        }
		Characterizer c1 = masterMap.at(p.GetPerspective());
		Characterizer c2 = R.GetCharacterizer();

		float sim = 0;

        // namedWindow("a");
        // imshow("a", c1.gs);
        // waitKey(0);
        // destroyAllWindows();

        // a combination of similarity tests:
        if(comboweights.size() != 3)
            ErrorIO("CompareAndWeight : Length of comboweight vector must be 3\n");

        if(!(abs(comboweights[0]) < .05))
            sim += CompareDescriptors(c1, c2) * comboweights[0];
        if(!(abs(comboweights[2]) < .05))
            sim += GetSimilarity(c1.gs, c2.gs) * comboweights[2];
        if(!(abs(comboweights[3]) < .05))
            sim += GetSimilarity(c1.bw, c2.bw) * comboweights[3];

        if (sim == -10000)
            ErrorIO("Descriptors couldn't be compared!");

        if (sim == -10000)
            sim = GetSimilarity(c1.gs, c2.gs);

        // cout
        // << "SURFS: " << 10 * (int) CompareDescriptors(c1.surfs, c2.surfs)
        // << "\tSIFTS: " << (int) CompareDescriptors(c1.sifts, c2.sifts) / 3
        // << "\tPXSUM: " << GetSimilarity(c1.pixSum, c2.pixSum)
        // << "\tABOVE: " << GetSimilarity(c1.bw, c2.bw) / 2
        // << "\tTOTAL: " << sim
        // << endl;

        return sim;
	}


    // given two images of different size, return a similarity score
    float SimilarityOfDifferentSizedImages(Mat& mat1, Mat& mat2)
    {
    // TODO
        return 10000.0;
    }

    // given two sets of keypoints and descriptors, return a similarity score
    float CompareDescriptors(Characterizer c1, Characterizer c2)
    {
        // First match descriptors
        FlannBasedMatcher matcher;
        vector<DMatch> matches;

        Mat desc1 = c1.descs;
        Mat desc2 = c2.descs;

        double total = 0.0;
        double count = 0.0;

        vector<vector<DMatch> > vecmatches;
        float ratio = 0.75;

        if (desc1.empty() || desc2.empty() || desc2.cols < 2 || desc2.rows < 2)
        {
            // ErrorIO("Error in Matching.cpp->CompareDescriptors: At least one of the descriptors is empty!");
            return 0;//-10000;
        }

        float sim = 0;

        matcher.knnMatch(desc1, desc2, vecmatches, 2);

        for (int i = 0; i < vecmatches.size(); i++)
            if (vecmatches[i][0].distance < ratio * vecmatches[i][1].distance)
                matches.push_back(vecmatches[i][0]);

        // vector<Point2f> obj1;
        // vector<Point2f> scene2;

        for( int i = 0; i < matches.size(); i++ )
        {
            total += matches[i].distance;
            count += 1.0;
            sim += 1/(matches[i].distance+0.8);

            // obj1.push_back(c1.kps[matches[i].queryIdx].pt);
            // scene2.push_back(c2.kps[matches[i].trainIdx].pt);
        }

        sim = count - 10* total / (count + 1);
        
        // if (obj1.size() < 4)
        //     return -200;//-1000;

        // Mat H = findHomography( obj1, scene2, CV_RANSAC );

        // vector<Point2f> results;

        // perspectiveTransform(obj1, results, H);

        // double dev = 0.0;
        // for( int i = 0; i < matches.size(); i++ )
        // {
        //     Point2f a = results[i];
        //     Point2f b = scene2[i];
        //     Point2f diff = a - b;
        //     dev += sqrt(diff.x*diff.x + diff.y*diff.y);
        // }

        // Mat img_matches;
        // drawMatches( c1.image, c1.kps, c2.image, c2.kps,
        //        matches, img_matches);

        // if(matches.size() > 0)
        //     dev /= matches.size();


        float score = sim;// /*-0.1 * dev + 5 **/ pow(sim, 0.75); //sqrt(sqrt(sim*sim*sim));

        // stringstream ss;
        // ss << "Score: " << score << /*", dev: "<< dev << */", sim: " << sim;
        // namedWindow(ss.str());
        // imshow(ss.str(), img_matches);
        // waitKey(0);
        // destroyAllWindows();


        if (count < 2)
            ErrorIO("Less than 2 matches!");
        if (count < 2)
            return 0;//-10000;
        // cout << total / count << " ";
        return score;
    }

    // Elementwise disance of two images.
    float ElementWiseDistance (Mat& mat1, Mat& mat2)
    {
        if (mat1.size() != mat2.size())
        {
            stringstream ss;
            ss << "Error in Matching.h -> ElementWiseDistance! Matrices are not the same size!";
            ErrorIO(ss.str());
            return -1000;
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