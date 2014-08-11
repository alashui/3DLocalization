#ifndef MCL_GLOBAL_H_
#define MCL_GLOBAL_H_

/** 
*	@file   - Globals.h
*	@author - Alex Rich, John Allard. Summer 2014.
*	@info   - Holds the declaration of all the globals needed in the localization program. This includes the master map that 
*			stores all of the perspectives in the map and all of the characterizers associated with those perspectives. We also 
*			have a single large vector containing every perspective in the environment. These are the only two globals
*			needed by our program.
**/ 

#include "../Perspective.h"
#include "../Characterizer.h"
#include <vector>

namespace MCL {
	extern map<Perspective, Characterizer, ComparePerspectives> masterMap;
	extern std::vector<Perspective> perspectives;
}

#endif