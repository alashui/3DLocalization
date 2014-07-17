/**
*
*   @File   Main.cpp
*   @Author Alex Rich and John Allard, Summer 2014 @ HMC
*   @Info   Starts up the MCL algorithm by loading features, then passes control
*           of program to the controller object.
*
**/

#include "../Boot/BootUp.h"
#include "../Helpers/Globals/Globals.h"
#include "../MCL/Control/Controller.h"

#include <sstream>

using namespace std;

namespace MCL
{
    int PrintError(string error)
    {
        stringstream ss;
        ss << "Input Format Error : " << error << endl;
        ErrorIO(ss.str());
        return -1;
    }
    int main(int argc, char const *argv[])
    {
        if (argc != 2)
            return PrintError("Must pass in the Name of the Model Directory as agrv[1]");

        string modelName = argv[1];

        if (BootUp(modelName) < 0)
            return PrintError("Model Directory Name not found inside /3DLocalization/Data/ModelData/.");

        Controller control;

        control.init(modelName);

        while(control.SpinOnce())
        {
            stringstream ss;
            ss << "Generation " << control.GetActiveParticles().GetGeneration() << ": " << control.GetActiveParticles().GetAvgWeight() << endl;
            UserIO(ss.str());
        }

        return 0;
    }
}