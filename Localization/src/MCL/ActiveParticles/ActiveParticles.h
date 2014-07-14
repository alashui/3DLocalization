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
        vector<Particle> pList;
        unsigned generation;
        vector<float> weightHistory;
        vector<Perspective> guessHistory;
        vector<Perspective> distribution;

        Perspective makeGuess();
        float computeAvgWeight();

    public:
        ActiveParticles();
        ~ActiveParticles();

        int generateDistribution(int wantedSize = defaultDistributionSize);
        void generateParticles(int amount = pList.size());
        Perspective analyzeList();
        int move(float x, float y, float z, float turntimes); // e.g. turntimes = -1, -2, -3, 0, 1, 2, 3, ...

        vector<Particle> getParticleList() const;
        void setParticleList(vector<Particle>);

        vector<Perspective> getWeightHistory() const;

        vector<Perspective> getGuessHistory() const;

        vector<Perspective> getDistribution() const;
        void setDistribution(vector<Perspective>);

        Perspective getGuess() const;
        int getAvgWeight() const;
        int getGeneration() const;
        int numParticles() const;

        const int defaultDistrbutionSize;
    };
}