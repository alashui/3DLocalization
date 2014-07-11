/**
*	@File   - Particle.h
*	@Author - John Allard, Alex Rich. Summer 2014.
*	@Info   - This is the declaration of the Particle class, a class that is used extensively by the 3DLocalization program. This class is declared inside
*			of this file and defined inside of Particle.cpp. The Particle class will be used to represent a single guess as to where our robot could be in 
			the environment. This 'guess' data structure will contain general data about the perspective 
*
**/

#ifndef MCL_PARTICLE_H_
#define MCL_PARTICLE_H_

namespace MCL
{

	class Particle
	{

	private:
	//********---------------------********//
	//*****-- Private Member Fields --*****//
	//********---------------------********//
		MCL::Perspective perspective;

		float weight;

	//********------------------------********//
	//*****-- Private Member Functions --*****//
	//********------------------------********//


	public:
	//**********------------**********//
	//*****-- Public Functions --*****//
	//**********------------**********//

		//====Constructor====//
		Particle();

		Particle(Perspective);

		Particle(Perspective, float)

		//====Destructor====//
		~Particle();


		//====Get and Set====//
		bool SetPerspective(Perspective);

		MCL::Perspective getPerspective() const;

		float getPerspective(int) const;  // Throws std::logic_error if argument is < 0 or > 5

		float getWeight() const;

		bool setWeight(float);

	//********-----------------------------------------------*******//
	//*****-- Public Definitions, Constants, and other Fields--*****//
	//********-----------------------------------------------*******//
		const float weightmin;
		const float weightmax;
		const float defaultweight;
	};

}

#endif