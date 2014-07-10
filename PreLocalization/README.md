Pre-Localization Programs
=========================
---
###Overview


This directory contains two programs that must be run before any localization efforts can be made. These two programs are responsible for generating all of the information needed by our localization algorithms and storing this information in the right location for our localization program to find. Any attempts to run the localization program without first running these two will result in compilation errors.

---

#Perspective Generator

    Version 1.0. July 10th, 2014.


###Overview

The general purpose of this program is to take in 3D model and a file stating what points in that model the user wants images rendered from. It then renders of all of these images tothe */Data/ModelData* directory for the DatabaseGen program to use.
####Process
* Generate an Inputfile for the program. This file contains all of the information needed for this program to generate the images from different perspective in the model.
   * The first line of the file is the name of the subdirectory inside */Data/ModelData/* that contains all of the model data that the user wants images generated from.
   * The second line is the name of the .obj, .3ds, or .dae file inside the above directory that contains all of the vertex information.
   * The third line is 6 numbers separated by spaces.
      * minX maxX minY maxY zHeight Resolution
      * min/max are the [x, y] bounds of the map. zHeight is the height above the floor the images should be rendered at. Resolution is the spacing of the grid that the user wants the images taken along. 
   * An example file is *Inputfiles/PointGenInput.txt*.
* The program uses the above input file to generate all of the points in the map that an image is to be rendered from.
* A window will appear and will have a perspective from the map shown inside of it.
* Click on the screen to start the process of rendering the images.
   * The program will cycle through every single point that was previously defined and write them to the *Data/RenderedImages/* folder under a subdirectory that is the same name as the directory that contains your model data in the */Data/ModelData/* directory.
* When the screen stops cycling through images the program is done and you can move on to the running the Database Generator program.

####Dependencies
This program makes extensive use of OpenGL and some common OpenGL libraries. It also uses OpenCV to do the image writing.
* OpenGL 3.3 minimum.
* CMake 2.8 minimum
* Boost 1.4 minimum
* GLEW
* GLUT
* OpenCV 

---

#Database Generator

    Version 1.0. July 10th, 2014.


###Overview



