// @File - RobotIO.h
// @Author - Alex Rich, John Allard. Summer 2014
// @Purpose - This file defines all of the functionality that this program needs to communicate with an actor in the environment.
//            This includes publishing localation information to the actor that needs to be localized, and recieving movement commands
//            from the robot so that we know how to properly update the particle locations.


#ifndef MCL_ROBOTIO_H_
#define MCL_ROBOTIO_H_

#include <stdio.h>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include "RobotState.h"
#include "../IO/ProgramIO.h"
#include "../MCL/Control/Controller.h"

#include <sstream>
#include <cmath>

extern ros::NodeHandle node;
extern ros::Publisher data_publisher;
extern ros::Subscriber movement_subscriber;

extern const std::string publisher;
extern const std::string subscriber;


namespace MCL
{

    bool RobotInit(int arc, char ** argv, std::string publisher, std::string subscriber);

    bool PublishData(std::string str);

	void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
}



#endif