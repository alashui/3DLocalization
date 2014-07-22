3DLocalization
========================

#### Project Overview

This project is an small collection of programs that allow a general actor (most of the time a robot) to localize itself in a pre-mapped environment using image-based feature data and particle filtering via the [Monte Carlo Localization algorithm.](en.wikipedia.org/wiki/Monte_Carlo_localization).

Written by Alex Rich and John Allard at Harvey Mudd College during the summer of 2014. Special thanks to the following
* Professor Dodds, our mentor for this project. Without his wisdom, generosity, and  industry connection this project would not have been possible.
* The National Science Foundation, for funding this project and many other REU programs around the nation.
* Matterport Inc, and specifically Mike Beebe, the co-founder and current COO, for loaning us a state-of-the-art 3D imaging camera. Without their generosity building the 3D models required for this project wouls have been difficult if not impossible.

#### Furthur Information

[3DLocalization Website](http://jhallard.github.io/3DLocalization/)

Code Documentation - `/3DLocalization/Documentation/CodeDocumentation/`.

Group Research Paper - `/3DLocalization/Documentation/ResearchPaper/`.

User Manual - `/3DLocalization/Documentation/UserManual/`

#### Directory Overview
* Data - See `/Data/README.md` for more information.
* Documentation - Contains all of the documentation for this project, which is also listed above.
* Localization - Contains the main 3DLocalization program (MCL algorithm implementation) as well as some GUI programs to help with debugging and visualization of our results. See `/Localization/README.md` for more information.
* PreLocalization - Contains two programs that must be run prior to attempting to localize a robot inside an environment. Extensive information about this process can be found in the Documentation section, but in short the two programs inside this folder process the model of the environment for features, then stores this data for use during the Localization attempt.