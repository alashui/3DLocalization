/**
*
*   @File   - ActiveParticles.h
*   @Author - Alex Rich and John Allard. Summer 2014
*   @Info   - Header file for the ActiveParticles class. This class represents a data structure that holds a list of Particle objects
*             and functions to work on these Particles. This class will be used as an object inside the Controller class which runs the
*             MCL algorithm and the 3DLocalization program. 
*
**/

#ifndef MCL_ACTIVEPARTICLES_H_
#define MCL_ACTIVEPARTICLES_H_

#include "../../Helpers/Globals/Globals.h"
#include "../../Particle/Particle.h"
#include "../../IO/ProgramIO.h"
#include <boost/random/normal_distribution.hpp>
#include <boost/random.hpp>

// #include <boost/random/mersenne_twister.hpp>
// #include <boost/random/uniform_int_distribution.hpp>


#include <vector>
#include <cstdlib>
#include <cmath>
#include <fstream>

// #include "../../Helpers/Characterize.h"

using namespace std;
using namespace MCL;

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
        float ComputeAvgWeight(int);           // pretty self explanatory if you ask me..
        // Perspective Scatter(Perspective, float, int);
        Perspective Scatter(Perspective);

    public:

    /**** Public Functions ****/
        ActiveParticles();
        // ~ActiveParticles();

        //@Function - GenerateDistribution
        //@Purpose  - Take the list of active particles, consider their respective perspectives and weightings, and generate a 
        //            distribution of perspectives that when sampled will provide perspectives in areas that we think are more likely
        //            to contain our actor.
        bool GenerateDistribution(int wantedSize);
        bool GenerateDistribution();               // Calls the above with this->defaultDistibutionSize as an argument

        //@Function - GenerateParticles
        //@Purpose  - This function samples the distribution created in the generateDistribution(..) to refill the pList vector
        //            with guesses (which are hopefully more refined) as to where the actor can be in the environment. Makes @param amount
        //            samples
        bool GenerateParticles(int amount);
        bool GenerateParticles();          // Calls the above with this->pList.size() as an argument  

        //@Function - AnalyzeList()
        //@Purpose  - Called once by the Controller class each iteration of the MCL loop. It computes data about the particle list like
        //            the average weights or generating a list of best-guesses.
        Perspective AnalyzeList();

        // Helper function to computate the angle given [x y] coords
        float GetAngle(float, float);

        //@Function - SnapToGrid
        //@Purpose  - Take a point computed in the continuum of our space, and move (snap) it to the nearest defined grid location
        //            that we image and feature data from.
        bool SnapToGrid(Perspective*);

        //@Function - move(..)
        //@Purpose  - Rotate the particle roate degrees and then translate it along it direction vector ( [dx dy dz] )
        int Move(float tranlate, float rotate); 

    //***** Get and Set Functions for Private Members ****/
        vector<Particle> GetParticleList();
        void GetParticleList(vector<Particle>*);
        void SetParticleList(vector<Particle>);

        vector<float> GetWeightHistory();

        vector<Perspective> GetGuessHistory();

        vector<Perspective> GetDistribution();
        void SetDistribution(vector<Perspective>);

        Perspective GetGuess();   // Get the current best guess as to the actors position
        float GetAvgWeight();       // Get the average weight for the current particle list
        int GetGeneration();      // Get the iteration count of the algorithm, how many times we have re-sampled particles
        int NumParticles();
        void WritePoints();
        void WriteMeta(float);
       

        bool GetConstants(string);      // Get the grid density and rotation interval from the generated input file (Data/InputFiles/$ModelName/InputFile.txt)

        /** Constants and Defines **/
        int defaultDistributionSize;    // Default size of the distribution that we sample from (currently 800, should be more)
    };
}

#endif