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
        Perspective p;

    public:
        // RobotState();
        // ~RobotState();

        Characterizer GetCharacterizer();
        void SetCharacterizer(Characterizer);

        Perspective GetPerspective();
        void SetPerspective(Perspective);

        void GenerateCharacterizer(Mat&);
    };
}

#endif