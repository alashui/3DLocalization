/**
*	@File   - Controller.h
*	@Author - John Allard, Alex Rich. Summer 2014.
*	@Info   - This is the declaration of the Controller class for the 3D-Localization program. The definition of the class is in
*			  the correspdongin Controller.cpp file. The purpose of this class is to dictate the flow of control of our localization program.
*			  This class will hold all of the information as to the current, previous, and future states of the program. It will have full control over
*			  the transitioning from one state to the other. The main function that this class is built around is the Spin() function. This function is
*			  called in the main function (main.cpp) inside of a while loop. When this function is called the controller will know that it needs to run 
*			  through an entire iteration of the Monte Carlo Localization algorithm. This includes moving the robot, sampling, comparing, and weighing the
*			  particle, generating a distribution of likely perspective for the robot, and resampling according to this distribution. The majority of this
*			  functionality is not defined inside of this class, this class just calls other classes (Like Robot or Particle) and their member functions to
*			  run the program.
*
**/

#ifndef CONTROLLER_H_
#define CONTROLLER_H_


class Controller
{

private:
//********---------------------********//
//*****-- Private Member Fields --*****//
//********---------------------********//

//********------------------------********//
//*****-- Private Member Functions --*****//
//********------------------------********//
public:
//**********------------**********//
//*****-- Public Functions --*****//
//**********------------**********//

//====Constructor====//
	Controller();

//====Destructor====//
	~Controller();

//********-----------------------------------------------*******//
//*****-- Public Definitions, Constants, and other Fields--*****//
//********-----------------------------------------------*******//
};


#endif