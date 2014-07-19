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
#include <stdio.h>

using namespace std;
using namespace MCL;

int PrintError(string error)
{
    ErrorIO(error);
    return -1;
}

int main(int argc, char ** argv)
{
    MCL::show_debug_IO = true; // Show the debugging output, remove this for final version or cleaner terminal output.

    srand(time(0));
    
    ros::init(argc, argv, "Localizer"); // Init must be called before initializing the Controller class;

    if (argc != 2)
        return PrintError("Must pass in the Name of the Model Directory as agrv[1]");

    string modelName = argv[1];

    if (BootUp(modelName) < 0)
        return PrintError("Model Directory Name not found inside /3DLocalization/Data/ModelData/.");

    UserIO("Boot Up is Finished. Be ready to start your robot control program.");
    UserIO("Press any key to continue");
    getchar();
    Controller control;

    if (!control.init(modelName))
    {
        return PrintError("Could Not Properly Initialize Controller.");
        return -1;
    }

    DebugIO("Initialization Finished Successfully, Starting Main Loop");

    namedWindow("Robot Image");
    namedWindow("Top Match");
    // namedWindow("Weighted Average");
    char key = ' ';
    while(key != 'q' && key != 'Q')
    {
        if(!control.SpinOnce())
            continue;
        
        RobotState r = control.GetRobotState();
        stringstream ss;
        ss << "Generation : " << control.GetActiveParticles().GetGeneration() << ", AvgWeight :  " << control.GetActiveParticles().GetAvgWeight();
        ss <<",\n  Guess : " << r.GetGuessPerspective().ToString(); 
        ss <<",\n  Weighted Guess : " << r.GetWeightedPerspective().ToString(); 
        DebugIO(ss.str());
        // UserIO(ss.str());
        imshow("Top Match", masterMap.at(r.GetGuessPerspective()).image);
        imshow("Robot Image", r.GetCharacterizer().image);
        // if (masterMap.count(r.GetWeightedPerspective()))
            // imshow("Weighted Average", masterMap.at(r.GetWeightedPerspective()).image);
        key = waitKey(1000);
    }
    destroyAllWindows();

    DebugIO("Initiating Self-Destruct Sequence.. 5 ... 4 .. 3 .. 2 .. 1");
    DebugIO("Booooooom????");

    return 0;
}