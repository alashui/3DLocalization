/**
*
*	@File ActiveParticles.h
*
*
*
**/

#ifndef MCL_ACTIVEPARTICLES_H_
#define MCL_ACTIVEPARTICLES_H_

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

		int generateDistribution();
		void generateParticles(int amount);
		Perspective analyzeList();
		int move(float x, float y, float theta);

		vector<Particle> getParticleList();
		void setParticleList(vector<Particle>);

		vector<Perspective> getWeightHistory();

		vector<Perspective> getGuessHistory();

		vector<Perspective> getDistribution();
		void setDistribution(vector<Perspective>);

		Perspective getGuess();
		int getAvgWeight();
		int getGeneration();

	};
}