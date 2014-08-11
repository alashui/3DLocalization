/**
*
*   @File   Main.cpp
*   @Author Alex Rich and John Allard, Summer 2014 @ HMC
*   @Info   Main start-up file for the Localization program. This file serves to combine all of the separate parts of the localization
*           program and actually localize a robot in an environment! The function main() will start off by loading all of the
*           necessary data from the datbase, connecting to the robot, and then running the main MCL loop until the robot is localized
*           or an error occurs.
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

    // attempt to load all of the model data from the database.
    if (BootUp(modelName) < 0)
        return PrintError("Model Directory Name not found inside /3DLocalization/Data/ModelData/.");

    // at this points, your robot control program should be started and looking for our programs main MCL data publish, under
    // the name MCL_DATA_PUBLISHER. Once your robot control program is attempting to subscribe to our publisher, hit any key to continue.
    UserIO("Boot Up is Finished. Be ready to start your robot control program.");
    UserIO("Press any key to continue");
    getchar();
    Controller control;

    // We now initiate the controller, this will go through the process of connecting to the robot, generating the initial particles,
    // and allocating memory for various processes.
    if (!control.init(modelName))
    {
        return PrintError("Could Not Properly Initialize Controller.");
        return -1;
    }

    DebugIO("Initialization Finished Successfully, Starting Main Loop");

    // namedWindow("Top Match");
    // namedWindow("Weighted Average");
    char key = ' ';

    // Main MCL Loop! It will run until the robot sends an EXIT_FLAG (666) or the user presses Q in the OpenCV rinwo.
    while(key != 'q' && key != 'Q' && !control.ExitFlagSet())
    {   
        stringstream ss; ss.str("");
        static time_t tstart = time(0);
        DebugIO("Start Loop");
        RobotState r = control.GetRobotState();
        
        if(!control.SpinOnce())
            continue;

        if(control.ExitFlagSet())
            return -1;

        key = waitKey(4);
        time_t temp = time(0) - tstart;
        tstart = time(0);
        ss << "End of Loop -- " << temp << " ms ";
        DebugIO(ss.str());
                
        // if (masterMap.count(r.GetGuessPerspective()))
        //     imshow("Top Match", masterMap.at(r.GetGuessPerspective()).image);
        // if (masterMap.count(r.GetWeightedPerspective()))
        //     imshow("Weighted Average", masterMap.at(r.GetWeightedPerspective()).image);
        // waitKey(1);
    }
    destroyAllWindows();

    ros::shutdown();

    DebugIO("Initiating Self-Destruct Sequence.. 5 ... 4 .. 3 .. 2 .. 1");
    DebugIO("Booooooom????");

    return 0;
}