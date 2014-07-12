3D Robotic Localization
========================

## Using OpenCV and the Monte Carlo Localization algorithm.


#### Overview
This directory contains all of the necessary source and build files for the 3DLocalization programm. This program will try its best to localize an actor in an environment given a 3D map of that environment and a stream of camera images from the actor. It is important to note that this program cannot be ran unless the two programs inside the `PreLocalization` directory are run first. This program relies on image feature data that is precomputed by those programs to do the matching and particle weighing that are central to the Monte Carlo Localization algorithm. For more information on the code in this directory, see the `Documentation/CodeDocumentation.tex` file. For more information on the Monte Carlo Localization algorithm and using computer vision algorithms for feature detection and matching, see the `Documentation/ResearchPaper.tex` file.

#### Directory Layout
Layout of this directory is:

   * src
      * Particle
         * Particle.h
         * Particle.cpp
      * Robot
         * Init.h
         * IO.h
         * RobotState.h
         * RobotState.cpp
      * HelperStructs
         * Perspective.h
         * Characterizer.h
      * MCL
         * ParticleAnalysis
            * Matching.h
            * ParticleControl.h
         * Control
            * Controller.h
            * Controller.cpp
      * Main
         * Main.cpp
   * build
   * CMakeLists.txt
   * README.md