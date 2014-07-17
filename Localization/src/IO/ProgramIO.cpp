/**
*   @File   - ProgramIO.cpp
*   @Author - John Allard and Alex Rich. Summer 2014.
*   @Info   - This file contains the definitions of the Input/Output related functions defined in the ProgramIO.h file. for more information on the purpose
*           of these functions see either the code documentation of the ProgramIO.h file.
*
**/

#include "ProgramIO.h"


namespace MCL
{
    // vector of strings that contain all of the error messages passed to the ErrorIO function
    std::vector<std::string> error_log;
    std::vector<std::string> debug_log;
    const std::string red = "\033[1;31m";
    const std::string yellow = "\033[1;33m";
    const std::string reset = "\033[0m";
    bool show_debug_IO = false;

    // @Function - DebugIO
    // @Input    - SS object containing the output message
    // @Output   - The text in the SS object to the screen under a yellow color. Also saves to a debug logger.
    // @Purpose  - For use when debugging output needs to be displayed to the screen. The purpose is to save us from complicating our program
    //            with inline debugging output messages.
    void DebugIO(std::string ss)
    {
        if(show_debug_IO)
        {
            std::cout << yellow << "[ " << ss << " ]" << reset << std::endl;
        }

        debug_log.push_back(ss);
    }

    // @Function - UserIO
    // @Input    - SS object containing the output message
    // @Output   - The text in the SS object is output to the screen under a white color. 
    // @Purpose  - for use when the program needs to display simple information to the user, like loading times, or the progress of a certain task.
    void UserIO(std::string ss)
    {
        std::cout << "[ " << ss << " ]" << std::endl;
    }

    // @Function - ErrorIO
    // @Input    - An SS object containing a user defined error message. 
    // @Output   - Outputs the text inside the SS object to the screen under a deep red and bolded color. This will capture the users attention
    //            and make it extremely obvious when an error has occured. In the future we might want to add an extra argument to allow the user to
    //            log an error code with this message.
    void ErrorIO(std::string ss)
    {
        error_log.push_back(ss);
        std::cout  << red <<" Error : " << "[ "  << ss  << " ]" << reset << std::endl;
    }

    // @Function - getErrorLog
    // @Input    - None 
    // @Output   - Returns a vector of strings filled with every single error message that has been passed to ErrorIO during the course of the program.
    std::vector<std::string> getErrorLog()
    {
        return error_log;
    }

    // @Function - PrintErrorLog
    // @Input    - Nothing
    // @Output   - Nothing returned, outputs every single error message inside of the error_log vector to the screen in a specific format.
    void PrintErrorLog()
    {
        std::cout << red <<" Printing Error Log. " << error_log.size() << " Total Errors." << reset << std::endl;
        for(int i = 0; i < error_log.size(); i++)
        {
            std::cout << red <<" Error #" << i << " [ " << error_log[i] << " ] " << reset << std::endl;
        }
    }

    // @Function - getErrorLog
    // @Input    - None 
    // @Output   - Returns a vector of strings filled with every single error message that has been passed to ErrorIO during the course of the program.
    std::vector<std::string> getDebugLog()
    {
        return debug_log;
    }

    // @Function - PrintErrorLog
    // @Input    - Nothing
    // @Output   - Nothing returned, outputs every single error message inside of the error_log vector to the screen in a specific format.
    void PrintDebugLog()
    {
        std::cout << yellow <<" Printing Debug Log. " << debug_log.size() << " Total Debug Notes." << reset << std::endl;
        for(int i = 0; i < debug_log.size(); i++)
        {
            std::cout << yellow <<" Debug Note #" << i << " [ " << debug_log[i] << " ] " << reset << std::endl;
        }
    }

}