/**
* Blah
*
**/

#include "ActiveParticles.h"
#include <boost/random/mersenne_twister.hpp>

namespace MCL
{

	ActiveParticles::ActiveParticles() : generation(0), defaultDistributionSize(800) {}
	
	Perspective ActiveParticles::makeGuess()
	{
		// First Determine Location
		int avgx = 0;
		int avgy = 0;
		int avgz = 0;

		for (int i = 0; i < this->numParticles(); i++)
		{
			Particle p(this->pList[i]);
			avgx += p.getWeight() * p.getPerspective(0);
			avgy += p.getWeight() * p.getPerspective(1);
			avgz += p.getWeight() * p.getPerspective(2);
		}
		avgx = avgx / (this->avgWeight * this->numParticles);
		avgy = avgy / (this->avgWeight * this->numParticles);
		avgz = avgz / (this->avgWeight * this->numParticles);

		
		
		return;
	}
		
	float ActiveParticles::computeAvgWeight()
	{
		float total = 0;

		for (int i = 0; i < this->pList; i++)
		{
			total += this->pList[i].getWeight();
		}
		float avg = total/this->pList.size();
		this->weightHistory.push_back(avg);
		return avg;
	}

	int ActiveParticles::generateDistribution(int wantedSize)
	{
		// Randomly generate a distribution based on 
		// the weights of all elements in pList
		int totalWeight = this->getAvgWeight() * this->numParticles();

		this->distribution.clear();

		for (int i = 0; i < this->pList.size(); i++)
		{
			int num = (this->weight * wantedSize) / totalWeight;
			for (; num > 0; num--)
				this->distribution.push_back(this->pList[i].getPerspective());
		}
		return 0;
	}

	void ActiveParticles::generateParticles(int amount)
	{
		this->pList.clear();
		boost::random::uniform_int_distribution<> dist(0, this->distribution.size() - 1);

		for (int i = 0; i < amount; i++)
		{
			int rndIdx = dist(time(0));
			Perspective P = this->distribution[rndIdx];
			pList.push_back(Particle(P));
		}
	}
		
	Perspective ActiveParticles::analyzeList()
	{
		this->computeAvgWeight();
		this->generation++;
		return this->makeGuess();
	}

	int ActiveParticles::move(float x, float y, float theta)
	{
		// TODO
		return 0;
	}

	vector<Particle> ActiveParticles::getParticleList()
	{
		return this->pList;
	}

	void ActiveParticles::setParticleList(vector<Particle> pl)
	{
		this->pList = pl;
	}

	vector<Perspective> ActiveParticles::getWeightHistory()
	{
		return this->weightHistory;
	}

	vector<Perspective> ActiveParticles::getGuessHistory()
	{
		return this->guessHistory;
	}

	vector<Perspective> ActiveParticles::getDistribution()
	{
		return this->distribution;
	}
	
	void ActiveParticles::setDistribution(vector<Perspective> dist)
	{
		this->distribution = dist;
	}

	Perspective ActiveParticles::getGuess()
	{
		return this->guessHistory.back();
	}
	
	int ActiveParticles::getAvgWeight()
	{
		return this->weightHistory.back();
	}
	
	int ActiveParticles::getGeneration()
	{
		return this->generation;
	}

	int ActiveParticles::numParticles()
	
{		return this->pList.size();
	}
}