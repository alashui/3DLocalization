#ifndef MCL_GLOBAL_H_
#define MCL_GLOBAL_H_

#include "../Perspective.h"
#include "../Characterizer.h"
#include <vector>

namespace MCL {
	extern map<Perspective, Characterizer, ComparePerspectives> masterMap;
	extern std::vector<Perspective> perspectives;
}

#endif