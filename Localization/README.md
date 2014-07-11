Monte Carlo Localization
========================

Implementation of the Monte Carlo Localization algorithm that uses a database of images and their locations from within a 3D map to compute the current location of a robot. Layout of this directory is:

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