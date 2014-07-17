/**
*   @File   - Controller.h
*   @Author - John Allard, Alex Rich. Summer 2014.
*   @Info   - This is the declaration of the Controller class for the 3D-Localization program. The definition of the class is in
*             the correspdongin Controller.cpp file. The purpose of this class is to dictate the flow of control of our localization program.
*             This class will hold all of the information as to the current, previous, and future states of the program. It will have full control over
*             the transitioning from one state to the other. The main function that this class is built around is the Spin() function. This function is
*             called in the main function (main.cpp) inside of a while loop. When this function is called the controller will know that it needs to run 
*             through an entire iteration of the Monte Carlo Localization algorithm. This includes moving the robot, sampling, comparing, and weighing the
*             particle, generating a distribution of likely perspective for the robot, and resampling according to this distribution. The majority of this
*             functionality is not defined inside of this class, this class just calls other classes (Like Robot or Particle) and their member functions to
*             run the program.
**/

#ifndef MCL_CONTROLLER_H_
#define MCL_CONTROLLER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include "../ActiveParticles/ActiveParticles.h"
#include "../../Helpers/Globals/Globals.h"
#include "../../Helpers/Characterizer.h"
#include "../../Helpers/Perspective.h"
#include "../../Particle/Particle.h"
#include "../../IO/ProgramIO.h"
#include "../../Robot/RobotState.h"
#include "../Matching/Matching.h"


namespace MCL
{

    class Controller
    {

    private:
    //*****-- Private Member Fields --*****//
        ActiveParticles ap;           // Class that defines our list of active particles and functionality related to them.
        RobotState robot;             // Class that defines our knowledge of the current state of the robot trying to be localized
        vector<float> comboWeighting; // weighing amount for each different feature detection algorithm. 

        ros::Publisher mclDataPublisher;         // ros::Publisher that allows us to publish data to the robot control program
        ros::Subscriber robotMovementSubscriber; // Our Subscriber object that allows us to get robot movement commands from the user
                                                 // so that we can update our particles according to the robots movement.
        ros::Subscriber robotImageSubscriber;    // This continuously recieves images from the robot.
        ros::NodeHandle * rosNodePtr;            // Handle to the ros::Node that we will publish and subscribe under
        Mat nextImage;                           // Holds the current image from the robot image callback function for us to grab 
                                                 // as needed.

        const std::string MCL_PUBLISHER_NAME;// = "MCL_Publisher";
        const std::string ROBOT_IMAGE_PUBLISHER_NAME;// = "ROBOT_IMAGE_DATA";
        const std::string ROBOT_MOVEMENT_PUBISHER_NAME; // = ROBOT MOVEMENT DATA


    //*****-- Private Member Functions --*****//
        
        // Get an image from the robots publisher and put it into the RobotState class, the RobotState class
        // will then process the image for feature data
        bool UpdateRobotData();

        bool RobotInit(int arc, char ** argv);
        bool PublishData(int,std::string str);
        void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
        void MovementCallback(const std_msgs::String msg);

        bool publisherConnected();         // returns true if the robot has subscribed to our data publisher
        bool imageSubscriberConnected();   // returns true if we have subscribed to the robots image publisher
        bool movementSubscriberConnected();// returns true if we have subscribed to the robots movement publisher

        // Take the ActiveParticles class and the RobotState class, pass them to the matching function and assign weights to the particles.
        bool CompareFeatures(); 
                                
        // generate a distribution based on the location and weighting of the active particles. Sample from this
        // distribution to create a new list of active particles.
        bool GenDistributionAndSample(); 

        // Start by sending a movement command to the robot, then update every particle in the particle list accordingly
        bool MoveUpdate();

        // Called when the program needs to wait for another part of the program to do something.
        // the argument is a pointer to a boolean flag that this function will wait to be true before moving on.
        // The second optional argument determines how long the function will wait before giving up and returning.
        bool PauseState(bool *, float ); 
        bool PauseState(bool (Controller::*foo)(), float);



    public:
    //*****-- Public Functions --*****//

        // Initializes member fields and our ros node.
        Controller();

        // Deletes the ros::NodeHandle that is dynamically allocated in Controller()
         ~Controller();

        // Based off the ROS::spin() function. This function is called once per iteration of the main algorithm loop inside Main/main.cpp.
        // This function is publically accessible and it will go and individually call the private member functions that run the localization
        // algorithm in the correct order. The purpose of this function is to abstract the process of localization away from the user and just have 
        // them call this function, the class will handle the rest internally. 
        bool SpinOnce();

        bool init(string, int, char**);

        // ActiveParticles get and set
        ActiveParticles GetActiveParticles() const;      // Get the ActiveParticles class by value.
        void GetActiveParticles(ActiveParticles *);// Get the ActiveParticles class by pointer
        bool SetActiveParticles(ActiveParticles);

        // RobotState get and set
        RobotState GetRobotState() const;
        void GetRobotState(RobotState *);
        bool SetRobotState(RobotState);

    
    //*****-- Public Definitions, Constants, and other Fields--*****//
    };

}

#endif