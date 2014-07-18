#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>


#include <sstream>
#include <iostream>
#include <cmath>
#include <ctime>

using namespace std;
using namespace cv;
using namespace ros;
namespace enc = sensor_msgs::image_encodings;

image_transport::Publisher data_publisher;
ros::Publisher  movement_publisher; 
ros::Subscriber mcl_data_subscriber;

const std::string publish_image_data_under = "ROBOT_IMAGE_PUBLISHER";
const std::string mcl_data_publisher_name = "MCL_DATA_PUBLISHER";


vector<Mat> image_list;
vector<string>  image_names;


void publish_Move()
{
	float dist = (float) ((rand()%10)-5)/10.0; // -.5, .5
	float theta = (float) ((rand()%120)-60);          // -60 60
	stringstream ss;
	std_msgs::String msg;
	ss << "20" << " " << dist << " " << theta;
	msg.data = ss.str();
	movement_publisher.publish(msg);
}

 void MyDataCallback(const std_msgs::String msg)
{
	std::cout << "begin callback" << endl;
	string str = msg.data;

	if(str == "15")
		std::cout << "handshake recieved" << std::endl;
	else if(str == "25")
	{
		std::cout << "Movement Command Recieved, starting move" << std::endl;
		std_msgs::String msg;
		msg.data = "10";
		movement_publisher.publish(msg);
		publish_Move();
	}

	cout << "end callback " << endl;
}


bool LoadImages()
{
	for(int i = 0; i < image_names.size(); i++)
	{
		image_list.push_back(imread(image_names[i]));
	}
}

int main(int argc, char **argv)
{
	srand(time(0));
	ros::init(argc ,argv, "ROS_Publisher");
	NodeHandle node;
	image_transport::ImageTransport it(node);

    mcl_data_subscriber = node.subscribe(mcl_data_publisher_name, 4, MyDataCallback);
	
	movement_publisher = node.advertise<std_msgs::String>("ROBOT_MOVEMENT_PUBLISHER", 4);

	//data_publisher = node.advertise<sensor_msgs::ImageConstPtr&>(publish_image_data_under, 4);
	data_publisher = it.advertise(publish_image_data_under, 4, true);

    image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_1.5_0.4_0.5_0.86_0_.jpg");

	LoadImages();

	while(true)
	{
		int i = rand()%image_names.size();
		cv_bridge::CvImage out_msg;
		ros::Time scan_time = ros::Time::now();
		out_msg.header.stamp = scan_time;
		out_msg.header.frame_id = "robot_image";
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image = image_list[0];
		//cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_list[i], enc::BGR8);
		data_publisher.publish(out_msg.toImageMsg());
		ros::spinOnce();

		ros::Duration(0.1).sleep();
	}


}