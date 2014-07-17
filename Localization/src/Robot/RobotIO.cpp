#include "RobotIO.h"


namespace MCL
{
    ros::Publisher data_publisher;
    ros::Subscriber movement_subscriber;

    const std::string publish = "MCL_Publisher";
    const std::string subscriber = "ROBOT_DATA";

    void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr im2;
        try
        {
            im2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (...)
        {
            ErrorIO("ImageCall back conversion failed");
        }

        Controller::SetNextInputImage(im2->image);
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

        movement_subscriber = node.subscribe(subscriber, 2, ImageCallback);

        return ros::ok();
    }

    bool PublishData(int code, std::string str)
    {
        std_msgs::String msg;
        msg.data = str;
        data_publisher.publish(msg);
        MCL::DebugIO("Data Published to Robot");
    }
}