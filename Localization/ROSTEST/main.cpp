..#include "ros/ros.h"
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

 void DataCallback(const std_msgs::String msg)
{
	std::cout << msg << std::endl;
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
    mcl_data_subscriber = node.subscribe(mcl_data_publisher_name, 2, DataCallback);
	
	movement_publisher = node.advertise<std_msgs::String>("ROBOT_MOVEMENT_PUBLISHER", 4);

	//data_publisher = node.advertise<sensor_msgs::ImageConstPtr&>(publish_image_data_under, 4);
	data_publisher = it.advertise(publish_image_data_under, 4, true);

    image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_1.25_1.75_0.4_0_-1_0_.jpg");
    image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_1.75_9_0.4_0.49_-0.86_0_.jpg");
    image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_2.5_0.75_0.4_0_-1_0_.jpg");
    image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_-2_9_0.4_0.5_0.86_0_.jpg");
	image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_1.25_1.75_0.4_0_1_0_.jpg");
	image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_-1.75_9_0.4_-0.5_-0.86_0_.jpg");/*       _2.5_0.75_0.4_0_1_0_.jpg             _2_9_0.4_-0.5_-0.86_0_.jpg
	_1.25_1.75_0.4_-0.49_0.86_0_.jpg                  _-1.75_9_0.4_0.5_0.86_0_.jpg         _-2.5_0.75_0.4_-0.49_0.86_0_.jpg     _2_9_0.4_0.5_0.86_0_.jpg*/

	LoadImages();

	while(true)
	{
		int i = rand()%image_names.size();
		cv_bridge::CvImage out_msg;
		ros::Time scan_time = ros::Time::now();
		out_msg.header.stamp = scan_time;
		out_msg.header.frame_id = "robot_image";
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image = image_list[i];

		if(image_list[i].empty())
			std::cout << "Image list empty\n";

		// if(out_msg.image.empty())
		// 	std::cout << "Output image is empty";

		std_msgs::String msg;
		stringstream ss;
        ss << "Move bitch, get out the way" << std::endl;
        msg.data = ss.str();
        movement_publisher.publish(msg);

		//cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_list[i], enc::BGR8);
		data_publisher.publish(out_msg.toImageMsg());
		ros::spinOnce();
	}


}