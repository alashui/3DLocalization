/**
*   @File   - Particle.h
*   @Author - John Allard, Alex Rich. Summer 2014.
*   @Info   - This is the declaration of the Particle class, a class that is used extensively by the 3DLocalization program. This class is declared inside
*           of this file and defined inside of Particle.cpp. The Particle class will be used to represent a single guess as to where our robot could be in 
            the environment. This 'guess' data structure will contain general data about the perspective 
*
**/

#ifndef MCL_PARTICLE_H_
#define MCL_PARTICLE_H_

#include "../Helpers/Perspective.h"
#include "../IO/ProgramIO.h"

namespace MCL
{

    class Particle
    {

    private:

    //*****-- Private Member Fields --*****//
        MCL::Perspective perspective;         // the current perspective associated with the particle
        float weight;                         // the weighting assigned to the particle based on its match with the actors image.

    //*****-- Private Member Functions --*****//


    public:

    //*****-- Public Functions --*****//

        //====Constructor====//
        Particle();                              // Default constructor, initializes members to default values.
        Particle(Perspective);                   // Initialize with a perspective abject.
        Particle(Perspective, float);             // Initialize with a perspective and a weight.

        //====Destructor====//
        ~Particle();                             // Place holder for a destructor, not needed right now.
 

        //====Get and Set====//
        bool SetPerspective(Perspective);
        MCL::Perspective GetPerspective() const; // Get the perspective associated with this particle
        float GetPerspective(int) const;         // Throws std::logic_error if argument is < 0 or > 5
        
        float GetWeight() const;                 // Get the current weight associated with this particle
        bool SetWeight(float);                   // Set the weighting for the particle

        float Distance(Particle);                // Get the Euclidean distance from one particle to this one

    //********-----------------------------------------------*******//
    //*****-- Public Definitions, Constants, and other Fields--*****//
        
         float weightmin;                   // minimum weight that can be assigned to a particle
         float weightmax;                    // maximum weight that can be assigned to a particle
         float defaultweight;               // default weight for a particle (current 10, out of 100)
    };

}

#endif