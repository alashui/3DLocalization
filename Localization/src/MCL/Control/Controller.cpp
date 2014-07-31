/**
*   @File    - Controller.cpp
*   @Author  - John Allard, Alex Rich. Summer 2014.
*   @Purpose - Implementation of the controller class defined in Controller.h. This class serves as the control mechanism for the localization
*              program.
**/

#define GRIDCONVERSION 1.125 // units of model space / meter

#include "Controller.h"

namespace MCL
{
    Controller::Controller() :
    defaultParticleListsize(200),
    MCL_PUBLISHER_NAME("MCL_DATA_PUBLISHER"), // name of the MCL-related data publisher that we must publish under.
    ROBOT_MOVEMENT_PUBLISHER_NAME("ROBOT_MOVEMENT_PUBLISHER"), // Name of the robot movement data publisher we must subscribe to and the user must publish under
    ROBOT_IMAGE_PUBLISHER_NAME("ROBOT_IMAGE_PUBLISHER"),
    starting_move(10),
    finished_move(20),
    handshake(15),
    readymove(25),
    robotdata(35),
    killflag(666)
    {
        rosNodePtr= new ros::NodeHandle();   // now throw the node handle on the stack
        comboWeighting.push_back(1.0); //SURF
        comboWeighting.push_back(0.0); //GREYSCALE
        comboWeighting.push_back(0.0); // B&W
        moving = false;
        image_feed_started = false;
        EXIT_FLAG = false;
    }

    Controller::~Controller()
    {
        delete rosNodePtr;
    }

    bool Controller::UpdateRobotData()
    { 
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

        float maxw = -1000;
        float minw = 10000;

        map<Perspective, float, ComparePerspectives> cachedPs;

        for(int i = 0; i < this->ap.NumParticles(); i++)
        {               
            Perspective cp = v[i].GetPerspective();
            if (!cachedPs.count(v[i].GetPerspective()))
                cachedPs[cp] = CompareAndWeigh(v[i], this->robot, this->comboWeighting);
            float wt = cachedPs[cp];

            v[i].SetWeight(wt);
            if (wt > maxw)
            {
                maxw = wt;
                robot.SetGuessPerspective(v[i].GetPerspective());
            }
            else if (wt < minw)
                minw = wt;
        }

        float w = maxw - minw;
        this->ap.SetParticleList(v);
        this->ap.WriteMeta();


        cout << "MAX: " << maxw << ", MIN: " << minw << endl;

        for (int i = 0; i < this->ap.NumParticles(); i++)
        {
            float currentw = v[i].GetWeight();
            v[i].SetWeight((currentw - minw) * 100.0 / w);
        }

        this->ap.SetParticleList(v);
    }
                            
    // generate a distribution based on the location and weighting of the active particles. Sample from this
    // distribution to create a new list of active particles.
    bool Controller::GenDistributionAndSample()
    {
        if(!ap.GenerateDistribution())
            return false;

        if(!ap.GenerateParticles(ap.GetParticleList().size()))
            return false;

        return true;
    }

    // Start by sending a movement command to the robot, then update every particle in the particle list accordingly
    bool Controller::MoveUpdate()
    {
        recentMove.clear();
        bool (Controller::*connect_flag) ();
        connect_flag = &Controller::robotIsMoving;

        //publish the ready-move command to the robot program.
        std_msgs::String msg;
        stringstream ss;
        ss << readymove << " ";
        msg.data = ss.str();
        mclDataPublisher.publish(msg);

        //ros::spinOnce();

        bool started = PauseState(connect_flag, 5);

        if(!started)
        {
            ErrorIO("MoveUpdate - robot did not send start_move Signal");
            this->ap.Move(0, 0);
            return false;
        }
        else
        {
            DebugIO("Robot has Started Moving");
            ros::spinOnce();
            while(moving)
            {
                ros::spinOnce();
            }
            DebugIO("Robot has finished moving");
            this->ap.Move(recentMove[0] * GRIDCONVERSION, recentMove[1]);
            return true;
        }
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
            ros::spinOnce();
            if(*flag)
                return true;
        }
    
        return false;
    }

    bool Controller::PauseState(bool (Controller::*foo)(), float waittime = 10.0)
    {
        ros::spinOnce();
        time_t temp = time(NULL);
        int i = 0;
        // Wait until the flag is true to return
        while((time(NULL) - temp) < waittime)
        {
            if(i%5 == 0) ros::spinOnce();
            if((this->*foo)())
                return true;
        }
        ros::spinOnce();
    
        return false;
    }
    bool Controller::init(string dirName)
    {
        // Initiate the ros::Callback functions and shake hands with the robot.
        if(!RobotInit())
        {
            ErrorIO("RobotInit failed");
            return false;
        }

        // Initiate our RobotState object with the data recieved from the robot
        UpdateRobotData();


        if(!this->ap.GetConstants(dirName))
            return false;

        this->ap.SetDistribution(perspectives);

        if(!this->ap.GenerateParticles(defaultParticleListsize))
            return false;

        this->ap.AnalyzeList();

        // For MetaData!
        float minx = 10000;
        float maxx = -10000;
        float miny = 10000;
        float maxy = -10000;

        for (int i = 0; i < perspectives.size(); i++)
        {
            Perspective p = perspectives[i];
            minx = p.x < minx ? p.x : minx;
            maxx = p.x > maxx ? p.x : maxx;
            miny = p.y < miny ? p.y : miny;
            maxy = p.y > maxy ? p.y : maxy;
        }

        ofstream mdFile;
        mdFile.open("../src/GUI/Meta/MetaData.txt");
        mdFile << minx << " " << maxx << " " << miny << " " << maxy << "\n";
        mdFile.close();

        return true;
    }

    bool Controller::SpinOnce()
    {

        time_t tstart = time(0);

        ros::spinOnce();

        CompareFeatures();

        time_t duration = time(0) - tstart;
        stringstream ss;
        ss << "[Controller] CompareFeatures took " << duration << " seconds.";
        DebugIO(ss.str());
        tstart = time(0);
        ss.str("");


        this->ap.AnalyzeList();
        robot.SetWeightedPerspective(ap.GetGuess());

        this->PublishData(robotdata, " ");

        duration = time(0) - tstart;
        ss << "[Controller] AnalyzeList took " << duration << " seconds.";
        DebugIO(ss.str());
        tstart = time(0);
        ss.str("");

        ros::spinOnce();
        
        if(!GenDistributionAndSample())
        {
            ErrorIO("[Controller] Failed to generate distribution and sample new particles");
            this->EXIT_FLAG = true;
            return false;
        }

        duration = time(0) - tstart;
        ss << "[Controller] GenDistributionAndSample took " << duration << " seconds.";
        DebugIO(ss.str());
        tstart = time(0);
        ss.str("");

        ros::spinOnce();
        
        if(!MoveUpdate())
        {
            ErrorIO("[Controller] MoveUpdate Failed");
            return false;
        }

        duration = time(0) - tstart;
        ss << "[Controller] MoveUpdate took " << duration << " seconds.";
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
        //DebugIO("Recieved Img from Robot");
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
        {
             if(!image_feed_started)
                image_feed_started = true;

            this->nextImage = im2->image;
        }
        else
            ErrorIO("Image Data from Robot is Null");
    }

    // Parse the string that contains the 4 movement commands, [x y z dtheta] with theta being in the xy plane
    void Controller::MovementCallback(const std_msgs::String msg)
    {
        //DebugIO("Inside of movement callback");


         std::string str = msg.data;
        // stringstream ss;
        // ss << "Recieved move command " << str;
        // DebugIO(ss.str());

        int state;

        std::vector<std::string> strs;
        boost::split(strs, str, boost::is_any_of(" _,"));

        state = atoi(strs[0].c_str());  // parse token to int

        if(state == killflag)
        {
            EXIT_FLAG = true;
            return;
        }
        //std::cout << "State value from robot : " << state << std::endl;
        if(state == starting_move)
        {
            moving = true;
            // DebugIO("Starting move value recieved");
            recentMove.clear();
            return;
        }
        else if(state == finished_move)
        {
            // DebugIO("Finished Move Value Recieved");
            moving = false;
            for(int i = 1; i < strs.size(); i++)
            {
              float value = atof(strs[i].c_str());
              recentMove.push_back(value);
            }
            // std::cout << "Movedetected : Translation = " << recentMove[0] << " || Rotation = " << recentMove[1] << std::endl;
        }
        else
        {
            stringstream ss; 
            ss << "Error : State command from robot must be " << starting_move << " (start move), "
            << finished_move << " (stop move), or " << killflag << " (killflag)";
            ErrorIO(ss.str());
        }


    }

    // Called when the robot program is 
    bool Controller::RobotInit()
    {
        stringstream ss;
        bool (Controller::*connect_flag) (); // fucntion pointer that our pause function will use to tell when we have
                                             // successfully connected to the robot

        DebugIO("Attempting to Subscribe to Robot ros::Publishers ... ");

        // Advertise our Data publisher
        mclDataPublisher = this->rosNodePtr->advertise<std_msgs::String>(MCL_PUBLISHER_NAME, 4);

        connect_flag = &Controller::publisherConnected;

        // Wait for the connection with our Data Publisher to succeed or time out.
        bool connection_succeded = this->PauseState(connect_flag, 30);

        if(!connection_succeded)
        {
            ErrorIO("Robot Program Failed to Subscribe to mclDataPublisher after 10 seconds. (RobotInit(..))");
            return false;
        }
        else
            DebugIO("Robot Has Subscribed to mclDataPublisher.");

        this->PublishData(handshake, "_");

        // pause for a second to let the robot start it's publishers.
        bool x = false; PauseState(&x, 2);

        // Subscribe to the robot movement data publisher.
        robotMovementSubscriber = this->rosNodePtr->subscribe(this->ROBOT_MOVEMENT_PUBLISHER_NAME, 2, &Controller::MovementCallback, this);

        // wait max 10 seconds for us to connect to the movement data publisher.
        connect_flag = &Controller::movementSubscriberConnected;
        connection_succeded = this->PauseState(connect_flag, 10);

        if(!connection_succeded)
        {
            ErrorIO("Robot Movement Publisher Failed to be Detected after 10 seconds of waiting. (RobotInit(..))");
            return false;
        }
        else
            DebugIO("Localization Program Has Subscribed to the Robot Movement Publisher.");

        // pause again for the robot to boot up the image callback.
        x = false; PauseState(&x, 2);

        // subscribe to the robot image publisher (implemented through the ros::ImageTransport class)
        image_transport::ImageTransport it(*rosNodePtr);
        robotImageSubscriber = it.subscribe(this->ROBOT_IMAGE_PUBLISHER_NAME, 2,&Controller::ImageCallback, this);

        // wait max 10 seconds for the subscriber to connect.
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

        // wait max 10 seconds for the subscriber to connect.
        connect_flag = &Controller::imageFeedStarted;
        connection_succeded = this->PauseState(connect_flag, 10);

        if(!connection_succeded)
        {
            ErrorIO("RobotInit() - Waited 30 Seconds for Robot to Publish Image Data But None Has Been Recieved");
            return false;
        }
        else
            DebugIO("Image data has been recieved");

        ros::spinOnce();

        return ros::ok();
    }

    // Publish a generatic string of data to the robot. the int code will be placed on front of the data string
    // for the robot to know how to process the command.
    bool Controller::PublishData(int code, std::string str)
    {
        std_msgs::String msg;
        stringstream ss;

        if(code == robotdata)
        {
            vector<float> temp = robot.GetGuessPerspective().ToVector();
            ss << code << "_" << temp[0] << "_" << temp[1] << "_" << temp[2] << "_" << temp[3] << "_" <<
             temp[4] << "_" << temp[5]; 
        }
        else
            ss << code << "_" << str;

        msg.data = ss.str();

        // publish the message
        mclDataPublisher.publish(msg);
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

    bool Controller::imageFeedStarted()
    {
        return(image_feed_started);
    }

    bool Controller::robotIsMoving()
    {
        return(moving);
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

    bool Controller::ExitFlagSet()
    {
        return this->EXIT_FLAG;
    }
}
