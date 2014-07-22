Test Robot
==========

###Purpose
This program represents a 'virtual' robot to test the 3DLocalization program with. This virtual robot with traverse a virtual 3D model of a specific environement instead of moving around in the actual environment, but besides that this program acts almost like the real thing. It fully implements all of the required interfaces of a real robot in order to work with the 3DLocalization program.
* This program will send rendered images from the 3D environement to the 3DLocalization program via a ROS publisher.
* This program will send movement commands to the program when requested.
* This program will subscribe to the data publisher from the 3DLocalization program and display the best-guess location for the virtual robot.

When you are trying to write a program for your own robot to interface with the 3DLocalization program, this is a good place to look for an example. Full documentation on the interface and functionality that your robot must implement in order to work with the 3DL program is described in the Official Code Documentation document, inside the `/3DLocalization/Documentation/` folder.