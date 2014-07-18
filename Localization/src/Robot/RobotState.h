/**
* 
*   @File RobotState.h
*   @Author Alex Rich and John Allard, Summer 2014
*   @Info A class to contain robot state information, continually updated.
*
**/

#ifndef MCL_ROBOTSTATE_H_
#define MCL_ROBOTSTATE_H_

#include "../Helpers/Characterizer.h"
#include "../Helpers/Perspective.h"

#include "../../../PreLocalization/DatabaseGen/src/AverageImage.h"

namespace MCL
{
    class RobotState
    {
    private:

        Characterizer c;
        Perspective pGuess;
        Perspective pWeighted;


    public:
        // RobotState();
        // ~RobotState();

        Characterizer GetCharacterizer();
        void SetCharacterizer(Characterizer);

        Perspective GetGuessPerspective();
        void SetGuessPerspective(Perspective);

        Perspective GetWeightedPerspective();
        void SetWeightedPerspective(Perspective);

        void GenerateCharacterizer(Mat&);
    };
}

#endif