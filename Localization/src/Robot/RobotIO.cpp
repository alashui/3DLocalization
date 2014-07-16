#include "RobotIO.h"


ros::NodeHandle node;
ros::Publish data_publisher;
ros::Subscriber movement_subscriber;

const std::string publisher = "MCL_Publisher";
const std::string subscriber = "MCL_Subscriber";


bool RobotInit(int arc, char ** argv)
{
    ros::init(argc, argv, "3DLocalization")
    data_publisher = node.advertise<std_msgs::String>(publish);

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