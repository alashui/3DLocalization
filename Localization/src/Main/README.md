Main Localization Program
=====

This directory contains the file main.cpp, which serves as the starting point for the Localization program. This file serves to load the data related to the map and the particles from the database into live variables, it then establishes a connection with the robot. Once this start-up proces is done, the main Monte Carlo Localization loop is started and run until the robot tells our program to stop. 