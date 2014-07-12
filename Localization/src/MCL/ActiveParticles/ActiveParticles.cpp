/**
* Blah
*
**/

#include "ActiveParticles.h"

namespace MCL
{
	Perspective ActiveParticles::makeGuess()
	{
		return NULL;
	}
		
	float ActiveParticles::computeAvgWeight()
	{
		return 0.0;
	}

	ActiveParticles::ActiveParticles();
	~ActiveParticles();

	int ActiveParticles::generateDistribution()
	{
		return 0;
	}
		
	void ActiveParticles::generateParticles(int amount)
	{
		return;
	}
		
	Perspective ActiveParticles::analyzeList()
	{
		this->computeAvgWeight();
		this->generation++;
		return this->makeGuess();
	}

	int ActiveParticles::move(float x, float y, float theta)
	{
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
}