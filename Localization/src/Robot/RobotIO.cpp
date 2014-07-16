#include "RobotIO.h"


namespace MCL
{
    ros::Publisher data_publisher;
    ros::Subscriber movement_subscriber;

    const std::string publish = "MCL_Publisher";
    const std::string subscriber = "MCL_Subscriber";

    void MotionCallback(const std_msgs::String::ConstPtr& msg)
    {
        int x = 2;
    }

    bool RobotInit(int argc, char ** argv)
    {
         ros::init(argc, argv, "3DLocalization");
       	 return true;
    }

    bool InitCallbacks()
    {
    	ros::NodeHandle node;
    	data_publisher = node.advertise<std_msgs::String>(publish, 4);

        std_msgs::String msg;
        std::stringstream ss;
        ss << "Initializing" << std::endl;
        msg.data = ss.str();

        data_publisher.publish(msg);

        movement_subscriber = node.subscribe(subscriber, 2, MotionCallback);

        return ros::ok();
    }

    bool PublishData(std::string str)
    {
        std_msgs::String msg;
        msg.data = str;
        data_publisher.publish(msg);
        MCL::DebugIO("Data Published to Robot");
    }
}