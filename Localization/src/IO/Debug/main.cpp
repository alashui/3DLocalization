#include "../../Particle/Particle.h"
#include "../ProgramIO.h"
#include <iostream>
#include <vector>


int main()
{
	MCL::ErrorIO("Error, returned null from GenerateParticles");
	MCL::ErrorIO("Error, invalid directory name as argv[1]");
	MCL::ErrorIO("Error : grid-density value is zero");
	MCL::ErrorIO("Error : Turning angle negative");
	MCL::ErrorIO("Fatal Error : Hang up in master loop, exiting 1");
	MCL::PrintErrorLog();
	MCL::DebugIO("Error encountered");

	return 0;

}