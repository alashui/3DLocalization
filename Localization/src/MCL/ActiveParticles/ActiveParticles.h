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
		ActiveParticles(arguments);
		~ActiveParticles();

		void generateDistribution();
		generateParticles(int amount);
		Perspective analyzeList();
		move(float x, float y, float theta);
	};
}