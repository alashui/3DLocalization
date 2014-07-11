/**
*	@File   - ProgramIO.h
*	@Author - John Allard and Alex Rich. Summer 2014.
*	@Info   - This is the declaration of the functions inside the IO namespace for the 3DLocalization program. The corresponding definitions of the functions
*			declared in this namespace are in ProgramIO.cpp. The purpose of this file is to store all of the functions that allow our program to accept and
*			send output to the user. This includes Debugging IO, General UserIO, and ErrorIO. This functions will format and log the IO information as
*			it comes through, allowing us to save important IO data and keep IO formatting consistant throughout the program.
*
**/

#ifndef 3DL_PROGRAMIO_H_
#define 3DL_PROGRAMIO_H_

#include <iostream>
#include <sstream>

namespace 3DL_io
{
	// vector of strings that contain all of the error messages passed to the ErrorIO function. Declared here and defined in the .cpp file to
	// avoid multiple inclusions.
	extern std::vector<std::string> error_log;

	// @Function - DebugIO
	// @Input    - SS object containing the output message
	// @Output   - The text in the SS object to the screen under a yellow color. Also saves to a debug logger.
	// @Purpose  - For use when debugging output needs to be displayed to the screen. The purpose is to save us from complicating our program
	// 			  with inline debugging output messages.
	void DebugIO(std::stringstream);

	// @Function - UserIO
	// @Input    - SS object containing the output message
	// @Output   - The text in the SS object is output to the screen under a white color. 
	// @Purpose  - for use when the program needs to display simple information to the user, like loading times, or the progress of a certain task.
	void UserIO(std::stringstream);

	// @Function - ErrorIO
	// @Input    - An SS object containing a user defined error message. 
	// @Output   - Outputs the text inside the SS object to the screen under a deep red and bolded color. This will capture the users attention
	//			  and make it extremely obvious when an error has occured. In the future we might want to add an extra argument to allow the user to
	//			  log an error code with this message.
	void ErrorIO(std::stringstream);

	// @Function - getErrorLog
	// @Input    - None 
	// @Output   - Returns a vector of strings filled with every single error message that has been passed to ErrorIO during the course of the program.
	std::vector<std::string> GetErrorLog();


}



#endif