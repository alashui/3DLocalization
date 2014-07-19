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
#include <boost/algorithm/string.hpp>

#include "boost/filesystem.hpp"

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

#include "../src/Helpers/Perspective.h"


#include <sstream>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <ctime>

using namespace std;
using namespace cv;
using namespace ros;
namespace enc = sensor_msgs::image_encodings;
namespace fs = ::boost::filesystem;

image_transport::Publisher data_publisher;
ros::Publisher  movement_publisher; 
ros::Subscriber mcl_data_subscriber;

const std::string publish_image_data_under = "ROBOT_IMAGE_PUBLISHER";
const std::string mcl_data_publisher_name = "MCL_DATA_PUBLISHER";

const int handshake = 15;
const int readymove = 25;
const int starting_move = 10;
const int finished_move = 20;


vector<Mat> image_list;
vector<string> image_names;
int current_image = 0;

vector<fs::path> ret;

bool handshake_recieved = false;

void GetAll(const fs::path& root, vector<fs::path>& ret);
bool loadImages();
vector<float> getMoveData();
float round(float x);

void publish_Move()
{
	vector<float> temp = getMoveData();
	stringstream ss;
	std_msgs::String msg;
	ss << "20" << " " << temp[0] << " " << temp[1] << " ";
	msg.data = ss.str();
	movement_publisher.publish(msg);
}

 void MyDataCallback(const std_msgs::String msg)
{
	string str = msg.data;

	if(atoi(str.c_str())== handshake)
	{
		handshake_recieved = true;
	}
	else if(atoi(str.c_str()) == readymove)
	{
		std::cout << "Movement Command Recieved, starting move" << std::endl;
		std_msgs::String msg;
		msg.data = "10";
		movement_publisher.publish(msg);
		ros::Duration(1.5).sleep();
		publish_Move();
	}
}


int main(int argc, char **argv)
{
	srand(time(0));
	ros::init(argc ,argv, "ROS_Publisher");
	NodeHandle node;
	image_transport::ImageTransport it(node);

	// std::cout << "Starting image load" << endl;
	// loadImages();
	// cout << "Done Loading Images" << endl;
	// getchar();

    mcl_data_subscriber = node.subscribe(mcl_data_publisher_name, 4, MyDataCallback);

    time_t temptime = time(0);
    std::cout << "Waiting for Handshake from Program .." << std::endl;
    while(!handshake_recieved && (time(0) - temptime) < 20)
    	ros::spinOnce();
    std::cout << "Handshake recieved" << std::endl;
	
	movement_publisher = node.advertise<std_msgs::String>("ROBOT_MOVEMENT_PUBLISHER", 4);

	//data_publisher = node.advertise<sensor_msgs::ImageConstPtr&>(publish_image_data_under, 4);
	data_publisher = it.advertise(publish_image_data_under, 4, true);

	 image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.25_1.75_0.4_1_0_0_.jpg");
	 image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_1.5_0.4_1_0_0_.jpg");
	 image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.25_1.75_0.4_1_0_0_.jpg");
	 image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_1_0.4_1_1_0_.jpg");
	 
	 for(int i = 0; i < image_names.size(); i++)
	 	image_list.push_back(imread(image_names[i]));

	//char key = 'k';
	// namedWindow("Robot Image");
	while(ros::ok())
	{
		ros::spinOnce();
		int i = rand()%image_names.size();
		cv_bridge::CvImage out_msg;
		ros::Time scan_time = ros::Time::now();
		out_msg.header.stamp = scan_time;
		out_msg.header.frame_id = "robot_image";
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image = image_list[0];
		// imshow("Robot Image", image_list[current_image]);

		// std::cout << image_list.size() << "\n";
		//cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_list[i], enc::BGR8);
		data_publisher.publish(out_msg.toImageMsg());
		ros::spinOnce();

		// key = cv::waitKey(2);
		// if(key == ' ')
		// 	current_image++;
		// if(current_image == image_list.size())
		// 	current_image = 0;
		//ros::Duration(0.1).sleep();
	}

}

	bool loadImages()
	{
		string dirName = "2ndFloorSprague/";
		string pathToData = "../../../Data";
        string toFeatures = pathToData + "/FeatureData/" + dirName;
        string toPhotos = pathToData + "/RenderedImages/" + dirName;

        string delimiter = "_";

        // Go into folder and get all available photos and features.
        const char * pstr = toPhotos.c_str();
        fs::path p(pstr);
        // Get a list of all filenames
        GetAll(pstr, ret);

        for(int i = 0; i < ret.size() && i < 1000; i++)
        {
        	stringstream ss;
        	ss << toPhotos << ret[i].string();
        	image_names.push_back(ss.str());
        	// std::cout << image_names[i] << endl;
        	image_list.push_back(imread(image_names[i]));
        }

        if(image_list.size() == 0)
        	cout << "failure!!" << endl;
        return true;

	}

	 void GetAll(const fs::path& root, vector<fs::path>& ret)
    {  
        if (!fs::exists(root)) return;

        if (fs::is_directory(root))
        {
            fs::recursive_directory_iterator it(root);
            fs::recursive_directory_iterator endit;
            while(it != endit)
            {
                if (fs::is_regular_file(*it))
                    ret.push_back(it->path().filename());
                ++it;
            }
        }
    }

    vector<float> getMoveData()
	{ 
		string str = image_names[current_image];

		std::vector<std::string> strs;
		std::vector<float> vals;
        boost::split(strs, str, boost::is_any_of(" _,"));

        for(int i = 0; i < 6; i++)
        {
        	vals.push_back(atof(strs[i].c_str()));
        	std::cout << vals[i] << endl;
        }

        float nangle = 0;
        nangle = atan2(vals[4], vals[3]) * 180.0 / 3.1415;

        std::cout << "atan" << endl;
        float theta = (float) ((rand()%2-4))*30;          // -60 60
		float dist = (float) ((rand()%3)-6)*0.25;

		theta += nangle;

		vals[0] += cos(nangle)*dist;
		vals[1] += sin(nangle)*dist;

		int xx = vals[0]/.25;
		vals[0] = xx*.25;

		xx = vals[1]/.25;
		vals[1] = xx*.25;

		int yy = theta/30;
		theta = yy*30;

		vals[3] = round(cos(theta));
		vals[4] = round(sin(theta));

		stringstream ss;
		ss << "_" << vals[0] << "_" << vals[1] << "_" << vals[2] << "_" << vals[3] << "_" << vals[4] << "_" << vals[5] << "_" << ".jpg";

		for(int i = 0; i < image_names.size(); i++)
		{
			if(image_names[i] == ss.str())
				current_image = i;
		}


        // MCL::Perspective P(vals);

        std::vector<float> ret;
        ret.push_back(dist);
        ret.push_back(theta);

        return ret;


	}

    float round(float x)
	{
	    return (float) ((int) (x*100))/100.0;
	}