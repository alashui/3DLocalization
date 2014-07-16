/**
*
* 	@File 	Main.cpp
* 	@Author Alex Rich and John Allard, Summer 2014 @ HMC
*	@Info 	Starts up the MCL algorithm by loading features, then passes control
*			of program to the controller object.
*
**/

#include "../Boot/BootUp.h"
#include "../Helpers/Globals/Globals.h"
#include "../MCL/Control/Controller.h"

using namespace std;

namespace MCL
{
	int PrintUsage(string appname)
	{
		cout << "Input not recognized. Format:\n\n"
		<< "\t" << appname << " modelName" << endl;
		return -1;
	}
	int main(int argc, char const *argv[])
	{
		if (argc != 2)
			return PrintUsage(argv[0]);

		string modelName = argv[1];

		if (BootUp(modelName) < 0)
			return PrintUsage(argv[0]);

		Controller control;
		control.spin();

		return 0;
	}
}