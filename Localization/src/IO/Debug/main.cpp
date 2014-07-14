#include "../../Particle/Particle.h"
#include "../ProgramIO.h"
#include <iostream>
#include <vector>


int main()
{
	MCL::ErrorIO(std::string("hello"));
	MCL::PrintErrorLog();
	MCL::DebugIO("Error encountered");

	return 0;

}