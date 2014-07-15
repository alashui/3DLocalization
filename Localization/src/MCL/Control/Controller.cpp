/**
*	@File    - Controller.cpp
*	@Author  - John Allard, Alex Rich. Summer 2014.
*	@Purpose - Implementation of the controller class defined in Controller.h. This class serves as the control mechanism for the localization
*			   program.
**/

	bool UpdateRobotData()
	{

	}

	// Take the ActiveParticles class and the RobotState class, pass them to the matching function and assign weights to the particles.
	bool CompareFeatures()
	{
		
	}
							
	// generate a distribution based on the location and weighting of the active particles. Sample from this
	// distribution to create a new list of active particles.
	bool GenDistributionAndSample(); 

	// Start by sending a movement command to the robot, then update every particle in the particle list accordingly
	bool MoveUpdate();

	// Called when the program needs to wait for another part of the program to do something.
	// the argument is a pointer to a boolean flag that this function will wait to be true before moving on.
	// The second optional argument determines how long the function will wait before giving up and returning.
	bool PauseState(bool *, float = 10.0); 