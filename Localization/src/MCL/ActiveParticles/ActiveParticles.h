/**
*
*   @File ActiveParticles.h
*   @Author Alex Rich and John Allard. Summer 2014
*   @Info Header for Active Particles
*
**/

#ifndef MCL_ACTIVEPARTICLES_H_
#define MCL_ACTIVEPARTICLES_H_

#include "../../Helpers/Globals/Globals.h"
#include "../../Helpers/Perspective.h"
#include "../../Particle/Particle.h"
#include <boost/random/mersenne_twister.hpp>

#include <vector>
#include <stdlib>
// #include "../../Helpers/Characterize.h"

using namespace std;

namespace MCL
{
    class ActiveParticles
    {

    private:
    /**** Private Fields ****/
        vector<Particle> pList;             // list of active particles
        unsigned generation;                // current iteration of the MCL algorithm loop
        vector<float> weightHistory;        // history of the average weight
        vector<Perspective> guessHistory;   // history of the best guesses for each MCL loop iteration
        vector<float> guessQualityHistory;  // history of guess quality
        vector<Perspective> distribution;   //   A vector of perspectives that is biased towards perspectives that are associated with a particle of
                                            // higher weight. When sampled randomly, this creates the effect of sampling from a distribution
                                            // that is skewed towards the areas in the map that are likely location for the actor 
        float gd;
        float dtheta;

    /**** Private Functions, Internal Use Only ****/
        Perspective MakeGuess();            // Computes the perspective that represents the best guess as to our actors location       
        float ComputeAvgWeight();           // pretty self explanatory if you ask me..

    public:

    /**** Public Functions ****/
        ActiveParticles();
        ~ActiveParticles();

        //@Function - generateDistribution
        //@Purpose  - Take the list of active particles, consider their respective perspectives and weightings, and generate a 
        //            distribution of perspectives that when sampled will provide perspectives in areas that we think are more likely
        //            to contain our actor.
        int GenerateDistribution(int wantedSize = defaultDistributionSize);

        //@Function - generateParticles
        //@Purpose  - This function samples the distribution created in the generateDistribution(..) to refill the pList vector
        //            with guesses (which are hopefully more refined) as to where the actor can be in the environment. Makes @param amount
        //            samples
        void GenerateParticles(int amount = pList.size());

        //@Function - analyzeList()
        //@Purpose  - Called once by the Controller class each iteration of the MCL loop. It computes data about the particle list like
        //            the average weights or generating a list of best-guesses.
        Perspective AnalyzeList();

        float GetAngle(float, float);
        void SnapToGrid(Perspective*);

        //@Function - move(..)
        //@Purpose  - Translate and rotate every particle by the given parameters. @param turntimes turns the object by n*theta, where
        //            theta is the rotation value between consecutive image renderings in the PerspectiveGenerator program.
        int Move(float x, float y, float z, float turntimes); // e.g. turntimes = -1, -2, -3, 0, 1, 2, 3, ...

        void GetConstants(string);

    //***** Get and Set Functions for Private Members ****/
        vector<Particle> GetParticleList() const;
        void GetParticleList(vector<Particles>*);
        void SetParticleList(vector<Particle>);

        vector<Perspective> GetWeightHistory() const;

        vector<Perspective> GetGuessHistory() const;

        vector<Perspective> GetDistribution() const;
        void SetDistribution(vector<Perspective>);

        Perspective GetGuess() const;
        int GetAvgWeight() const;
        int GetGeneration() const;
        int NumParticles() const;

        void GetConstants(string);
        float GetAngle(float, float);

        /** Constants and Defines **/
        const int defaultDistrbutionSize;
    };
}

#endif