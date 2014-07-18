/**
*   @File    - Controller.cpp
*   @Author  - John Allard, Alex Rich. Summer 2014.
*   @Purpose - Implementation of the controller class defined in Controller.h. This class serves as the control mechanism for the localization
*              program.
**/

#include "Controller.h"

namespace MCL
{
    Controller::Controller() :
    MCL_PUBLISHER_NAME("MCL_DATA_PUBLISHER"), // name of the MCL-related data publisher that we must publish under.
    ROBOT_MOVEMENT_PUBLISHER_NAME("ROBOT_MOVEMENT_PUBLISHER"), // Name of the robot movement data publisher we must subscribe to and the user must publish under
    ROBOT_IMAGE_PUBLISHER_NAME("ROBOT_IMAGE_PUBLISHER")
    {
        rosNodePtr= new ros::NodeHandle();   // now throw the node handle on the stack
        comboWeighting.push_back(1.0); //SURF
        comboWeighting.push_back(0.0); //SIFT
        comboWeighting.push_back(0.0); //GREYSCALE
        comboWeighting.push_back(0.0); // B&W
    }

    Controller::~Controller()
    {
        delete rosNodePtr;
    }

    bool Controller::UpdateRobotData()
    { 
        // TODO : Grab frame from Camera callback, put the image into the robotstate class. Have the robot class process the image.
        // Mat im = imread("../../../image.jpg");
        if(this->nextImage.empty())
        {
            ErrorIO("nextImage.Data is null in UpdateRobotData");
            return false;
        }
        else
            robot.GenerateCharacterizer(this->nextImage);
        return true;
    }

    // Take the ActiveParticles class and the RobotState class, pass them to the matching function and assign weights to the particles.
    bool Controller::CompareFeatures()
    {
        vector<Particle> v = this->ap.GetParticleList();

        float bestweight = -1;
        for(int i = 0; i < this->ap.NumParticles(); i++)
        {   
            float wt = CompareAndWeigh(v[i], this->robot, this->comboWeighting);
            v[i].SetWeight(wt);
            if (wt > bestweight)
            {
                bestweight = wt;
                robot.SetGuessPerspective(v[i].GetPerspective());
            }
        }

        this->ap.SetParticleList(v);
    }
                            
    // generate a distribution based on the location and weighting of the active particles. Sample from this
    // distribution to create a new list of active particles.
    bool Controller::GenDistributionAndSample()
    {
        ap.GenerateDistribution();
        ap.GenerateParticles();
    }

    // Start by sending a movement command to the robot, then update every particle in the particle list accordingly
    bool Controller::MoveUpdate()
    {
        float xmove, ymove, zmopve;
        int thetamove = 0;

    //***** TODO - Add some sort of randomization techniques for determining the movement commands *********//
        // We need to first move the robot.
        // RobotIO::PublishMovecommand(movex, movey, thetamove)

        // We then need to move the particles.
        // this->ap.Move(movex, movey, movez, thetamove);
    }

    // Called when the program needs to wait for another part of the program to do something.
    // the argument is a pointer to a boolean flag that this function will wait to be true before moving on.
    // The second optional argument determines how long the function will wait before giving up and returning.
    bool Controller::PauseState(bool * flag, float waittime = 10.0) //(seconds)
    {
       time_t temp = std::time(NULL);
        // Wait until the flag is true to return
        while((time(NULL) - temp) < waittime)
        {
            if(*flag)
                return true;
        }
    
        return false;
    }

    bool Controller::PauseState(bool (Controller::*foo)(), float waittime = 10.0)
    {
        ros::spinOnce();
        time_t temp = time(NULL);
        // Wait until the flag is true to return
        while((time(NULL) - temp) < waittime)
        {
            if((this->*foo)())
                return true;
        }
        ros::spinOnce();
    
        return false;
    }
    bool Controller::init(string dirName)
    {
        
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
// !!!!!!!!  THIS MIGHT CAUSE AN ERROR !!!!!!!!!! //
           RobotInit();
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
        UpdateRobotData();

        if(!this->ap.GetConstants(dirName))
            return false;
        this->ap.SetDistribution(perspectives);
        this->ap.GenerateParticles(600);
        this->ap.AnalyzeList();

        return true;
    }

    bool Controller::SpinOnce()
    {
        time_t tstart = time(0);

        ros::spinOnce();

        CompareFeatures();

        time_t duration = time(0) - tstart;
        stringstream ss;
        ss << "CompareFeatures took " << duration << " seconds.";
        DebugIO(ss.str());
        tstart = time(0);
        ss.str("");

        this->ap.AnalyzeList();
        robot.SetWeightedPerspective(ap.GetGuess());

        duration = time(0) - tstart;
        ss << "AnalyzeList took " << duration << " seconds.";
        DebugIO(ss.str());
        tstart = time(0);
        ss.str("");

        ros::spinOnce();
        
        GenDistributionAndSample();

        duration = time(0) - tstart;
        ss << "GenDistributionAndSample took " << duration << " seconds.";
        DebugIO(ss.str());
        tstart = time(0);
        ss.str("");

        ros::spinOnce();
        
        MoveUpdate();

        duration = time(0) - tstart;
        ss << "MoveUpdate took " << duration << " seconds.";
        DebugIO(ss.str());

        ros::spinOnce();

        UpdateRobotData();
        
        ros::spinOnce();

        return true;
    }


    //==========================================================================//
    // ====== ROBOT COMMUNICATION CALLBACKS AND INITIALIZATION FUNCTIONS ====== //
    //==========================================================================//

    // recieves and processes images that are published from the robot or other actor. It is up to the user to set up
    // their own publisher and to convert their image data to a sensor_msgs::Image::ConstPtr& .
    void Controller::ImageCallback(const sensor_msgs::Image::ConstPtr& img)
    {
        DebugIO("Recieved Img from Robot");
        cv_bridge::CvImagePtr im2;
        try
        {
            im2 = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        }
        catch (...)
        {
            ErrorIO("ImageCallback toCvCopy call failed");
        }

        if(!im2->image.empty())
            this->nextImage = im2->image;
        else
            DebugIO("Image Data from Robot is Null");

        //SetNextInputImage(im2->image);
    }

    // Parse the string that contains the 4 movement commands, [x y z dtheta] with theta being in the xy plane
    void Controller::MovementCallback(const std_msgs::String msg)
    {
        DebugIO("Inside of movement callback");
        std::string str = msg.data;
        return;
        float movement[4] = {0,0,0,0};
        int j = 0, count = 0, value = 0;
        for(int i = 0; count < 4 && i != str.size(); i++)
        {
            if(i == str.size()-1 || (str[i] == ' ' || str[i] == '_' || str[i] == ','))
            {
                std::string temp;
                temp = str.substr(j, i);
                value = atof(temp.c_str());
                movement[count] = value;
                count++;
                j = i;
            }
        }

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
//!!!!!!!TODO - We have the movment data from the robot, find a way to store it so that MovementUpdate function can use it!!!!!!!!//
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
    }

    bool Controller::publisherConnected()
    {
        return(this->mclDataPublisher.getNumSubscribers());
    }

    bool Controller::imageSubscriberConnected()
    {
        return(robotImageSubscriber.getNumPublishers());
    }

    bool Controller::movementSubscriberConnected()
    {
        return(robotMovementSubscriber.getNumPublishers());
    }

    // Called when the robot program is 
    bool Controller::RobotInit()
    {
        DebugIO("Attempting to Subscribe to Robot ros::Publishers ... ");
        mclDataPublisher = this->rosNodePtr->advertise<std_msgs::String>(MCL_PUBLISHER_NAME, 4);

        bool (Controller::*connect_flag) ();
        connect_flag = &Controller::publisherConnected;
        bool connection_succeded = this->PauseState(connect_flag, 10);

        stringstream ss;
        // ss << "Num subscribers : " << mclDataPublisher.getNumSubscribers();
        // DebugIO(ss.str());

        if(!connection_succeded)
        {
            ErrorIO("Robot Program Failed to Subscribe to mclDataPublisher after 10 seconds. (RobotInit(..))");
            return false;
        }
        else
            DebugIO("Robot Has Subscribed to mclDataPublisher.");

        // ++++ TODO - Send a Handshake greeting to the robot program - TODO ++++ //
        std_msgs::String msg;
        ss << "Initializing" << std::endl;
        msg.data = ss.str();
        mclDataPublisher.publish(msg);
        // ++++ TODO - Send a Handshake greeting to the robot program - TODO ++++ //

        bool x = false;
        PauseState(&x, 3);

        robotMovementSubscriber = this->rosNodePtr->subscribe(this->ROBOT_MOVEMENT_PUBLISHER_NAME, 2,
        &Controller::MovementCallback, this);

        // wait max 5 seconds for the subscriber to connect.
        connect_flag = &Controller::movementSubscriberConnected;
        connection_succeded = this->PauseState(connect_flag, 10);

        if(!connection_succeded)
        {
            ErrorIO("Robot Movement Publisher Failed to be Detected after 10 seconds of waiting. (RobotInit(..))");
            return false;
        }
        else
            DebugIO("Localization Program Has Subscribed to the Robot Movement Publisher.");


        x = false;
        PauseState(&x, 3);
        image_transport::ImageTransport it(*rosNodePtr);
        robotImageSubscriber = it.subscribe(this->ROBOT_IMAGE_PUBLISHER_NAME, 2,
         &Controller::ImageCallback, this);

        // wait max 5 seconds for the subscriber to connect.
        connect_flag = &Controller::imageSubscriberConnected;
        connection_succeded = this->PauseState(connect_flag, 10);

        if(!connection_succeded)
        {
            ErrorIO("Robot Image Publisher Failed to be Detected after 10 seconds of waiting. (RobotInit(..))");
            return false;
        }
        else
            DebugIO("Successfully Connected to robot Image Feed");

        ros::spinOnce();
        ros::spinOnce();
        return ros::ok();
    }

    bool Controller::PublishData(int code, std::string str)
    {
        std_msgs::String msg;
        stringstream ss;

        // turn the integer code and the string of data into a ros String message
        ss << code << "_" << str;
        msg.data = ss.str();

        // publish the message
        mclDataPublisher.publish(msg);

        // alert the user of the publishing
        MCL::DebugIO("Data Published to Robot");
    }


     // ActiveParticles get and set
    ActiveParticles Controller::GetActiveParticles() const      // Get the ActiveParticles class by value.
    {
        return this->ap;
    }
    void Controller::GetActiveParticles(ActiveParticles * ap2) // Get the ActiveParticles class by pointer
    {
        ap2 = &(this->ap);
    }
    bool Controller::SetActiveParticles(ActiveParticles ap2)
    {
        this->ap = ap2;
    }

    // RobotState get and set
    RobotState Controller::GetRobotState() const
    {
        return this->robot;
    }
    void Controller::GetRobotState(RobotState * newbot)
    {
        newbot = &(this->robot);
    }
    bool Controller::SetRobotState(RobotState rs)
    {
        this->robot = rs;
    }
}
