3D Robotic Localization Using CV and MCL.
=====


#### Overview
This directory contains all of the necessary source and build files for the 3DLocalization programm. This program will try its best to localize an actor in an environment given a 3D map of that environment and a stream of camera images from the actor. It is important to note that this program cannot be ran unless the two programs inside the `PreLocalization` directory are run first. This program relies on image feature data that is precomputed by those programs to do the matching and particle weighing that are central to the Monte Carlo Localization algorithm. For more information on the code in this directory, see the `Documentation/CodeDocumentation.tex` file. For more information on the Monte Carlo Localization algorithm and using computer vision algorithms for feature detection and matching, see the `Documentation/ResearchPaper.tex` file.

#### Directory Layout

   * src
      * Particle
         * Particle.h
         * Particle.cpp
      * Robot
         * RobotInit.h
         * RobotIO.h
         * RobotState.h
         * RobotState.cpp
      * HelperStructs
         * Perspective.h
         * Characterizer.h
      * View
         * View.h
         * View.cpp
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

#### Notes
* To run this program your robot/actor control program must implement a predefined interface that allows our program to send the commands and recieve the images it needs to run properly.
   * This is a simple interface built around the ROS libraries.
   * In short, you program must publish image data via a `ROS::Publisher`, subscribe to our command feed via a `ROS::subscriber`, and a few other minor things.
   * Extensive information about this interface can be found in the `Documentation/CodeDocumentation.tex` file or the `Documentation/UserManual.tex` file.
* Build files are excluded from the project via a .gitignore file.
* Each program will have a list of dependecies states inside of the `Documentation/CodeDocumentation.tex` file, check the version numbers of the required libraries to ensure compatibility.
* 