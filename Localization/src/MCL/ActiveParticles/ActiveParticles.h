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
#include "../../IO/ProgramIO.h"
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>


#include <vector>
#include <cstdlib>
#include <cmath>

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
        float gd;                           // The grid-density, aka the spacing along either the [x y z] axis between grid intersection points.
        float dtheta;                       // the angle of rotation between consectutive perspectives at the same [x y z] point in space

    /**** Private Functions, Internal Use Only ****/
        Perspective MakeGuess();            // Computes the perspective that represents the best guess as to our actors location       
        float ComputeAvgWeight();           // pretty self explanatory if you ask me..

    public:

    /**** Public Functions ****/
        ActiveParticles();
        ~ActiveParticles();

        //@Function - GenerateDistribution
        //@Purpose  - Take the list of active particles, consider their respective perspectives and weightings, and generate a 
        //            distribution of perspectives that when sampled will provide perspectives in areas that we think are more likely
        //            to contain our actor.
        int GenerateDistribution(int wantedSize);
        int GenerateDistribution();               // Calls the above with this->defaultDistibutionSize as an argument

        //@Function - GenerateParticles
        //@Purpose  - This function samples the distribution created in the generateDistribution(..) to refill the pList vector
        //            with guesses (which are hopefully more refined) as to where the actor can be in the environment. Makes @param amount
        //            samples
        void GenerateParticles(int amount);
        void GenerateParticles();          // Calls the above with this->pList.size() as an argument  

        //@Function - AnalyzeList()
        //@Purpose  - Called once by the Controller class each iteration of the MCL loop. It computes data about the particle list like
        //            the average weights or generating a list of best-guesses.
        Perspective AnalyzeList();

        // Helper function to computate the angle given [x y] coords
        float GetAngle(float, float);

        //@Function - SnapToGrid
        //@Purpose  - Take a point computed in the continuum of our space, and move (snap) it to the nearest defined grid location
        //            that we image and feature data from.
        void SnapToGrid(Perspective*);

        //@Function - move(..)
        //@Purpose  - Translate and rotate every particle by the given parameters. @param turntimes turns the object by n*theta, where
        //            theta is the rotation value between consecutive image renderings in the PerspectiveGenerator program.
        int Move(float x, float y, float z, float turntimes); // e.g. turntimes = -1, -2, -3, 0, 1, 2, 3, ...

    //***** Get and Set Functions for Private Members ****/
        vector<Particle> GetParticleList() const;
        void GetParticleList(vector<Particle>*);
        void SetParticleList(vector<Particle>);

        vector<Perspective> GetWeightHistory() const;

        vector<Perspective> GetGuessHistory() const;

        vector<Perspective> GetDistribution() const;
        void SetDistribution(vector<Perspective>);

        Perspective GetGuess() const;   // Get the current best guess as to the actors position
        int GetAvgWeight() const;       // Get the average weight for the current particle list
        int GetGeneration() const;      // Get the iteration count of the algorithm, how many times we have re-sampled particles
        int NumParticles() const;       

        void GetConstants(string);      // 

        /** Constants and Defines **/
        int defaultDistributionSize;    // Default size of the distribution that we sample from (currently 800, should be more)
    };
}

#endif