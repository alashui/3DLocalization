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

string dirName = "2ndFloorSprague/";
string pathToData = "../../../Data";
string toFeatures = pathToData + "/FeatureData/" + dirName;
string toPhotos = pathToData + "/RenderedImages/" + dirName;

const int handshake = 15;
const int readymove = 25;
const int guessdata = 35;
const int killflag = 666;
const int starting_move = 10;
const int finished_move = 20;

const int num_images = 2000;


vector<Mat> image_list;
vector<string> image_names;
int current_image = 0;
Mat BestGuessImage = imread("../../../Data/RenderedImages/2ndFloorSprague/_0.5_3.25_0.4_0_-1_0_.jpg");

vector<fs::path> ret;

bool handshake_recieved = false;

void GetAll(const fs::path& root, vector<fs::path>& ret);
bool loadImages();
vector<float> getMoveData();
float round(float x);

void publish_Move()
{
	static int count = 0;
	vector<float> temp = getMoveData();
	stringstream ss;
	std_msgs::String msg;
	msg.data = "10";
	movement_publisher.publish(msg);
	ros::spinOnce();
	ros::Duration(1).sleep();


	ss << "20" << " " << temp[0] << " " << temp[1] << " ";
	msg.data = ss.str();
	movement_publisher.publish(msg);

	if(current_image == image_list.size())
		current_image = 0;
	// imshow("Robot Image", image_list[current_image]);

}

 void MyDataCallback(const std_msgs::String msg)
{
	string str = msg.data;
	std::vector<std::string> strs;
	std::vector<float> vals;
    boost::split(strs, str, boost::is_any_of("_"));


    int command = atoi(strs[0].c_str());

	if(command == handshake)
	{
		handshake_recieved = true;
	}
	else if(command == readymove)
	{
		std::cout << "Movement Command Recieved, starting move" << std::endl;
		publish_Move();
	}
	else if(command == guessdata)
	{
		cout << "Guess data : " << str <<endl;
		for(int i = 0; i < strs.size(); i++)
		{
			cout << " " << strs[i] << " ";
		}
		cout << endl;
		if(strs.size() != 7)
		{
			cout << "Invalid guess data format" << endl;
			return;
		}
		vector<float> vars;
		float weight;

		for(int i = 1; i < 7 && i < strs.size(); i++)
		{
			vars.push_back(atof(strs[i].c_str()));
		}

		stringstream ss;
		ss << toPhotos << "_" << vars[0] << "_" << vars[1] << "_" << vars[2] << "_" << vars[3] << "_" << vars[4] << "_" << vars[5] << "_.jpg";

		BestGuessImage = imread(ss.str());

		if(BestGuessImage.empty())
		{
			cout << "image data corrupted in guess data callback" << endl;
			BestGuessImage = imread("../../../Data/RenderedImages/2ndFloorSprague/_0.5_3.25_0.4_0_-1_0_.jpg");
			return;
		}
	}
}


int main(int argc, char **argv)
{
	srand(time(0));
	ros::init(argc ,argv, "ROS_Publisher");
	NodeHandle node;
	image_transport::ImageTransport it(node);

	std::cout << "Starting image load" << endl;
	loadImages();
	cout << "Done Loading Images" << endl;
	getchar();

    mcl_data_subscriber = node.subscribe(mcl_data_publisher_name, 4, MyDataCallback);

    time_t temptime = time(0);
    std::cout << "Waiting for Handshake from Program .." << std::endl;
    while(!handshake_recieved && (time(0) - temptime) < 20)
    {
    	ros::spinOnce();
    }
    if(handshake_recieved)
    	std::cout << "Handshake recieved" << std::endl;
    else
    {
    	std::cout << "No handshake recieved";
    	return -1;
    }
	
	movement_publisher = node.advertise<std_msgs::String>("ROBOT_MOVEMENT_PUBLISHER", 4);

	data_publisher = it.advertise(publish_image_data_under, 4, true);


	char key = 'k';
	namedWindow("Robot Image");
	namedWindow("Top Match");

	while(ros::ok() && key != 'q')
	{
		ros::spinOnce();
		int i = rand()%image_names.size();
		cv_bridge::CvImage out_msg;
		ros::Time scan_time = ros::Time::now();
		out_msg.header.stamp = scan_time;
		out_msg.header.frame_id = "robot_image";
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image = image_list[current_image];
		
		if(!out_msg.image.empty())
			data_publisher.publish(out_msg.toImageMsg());
		ros::spinOnce();

		imshow("Robot Image", image_list[current_image]);
		imshow("Top Match", BestGuessImage);

		key = cv::waitKey(2);
		if(key == ' ')
			current_image++;
		if(current_image == image_list.size())
			current_image = 0;
		//ros::Duration(0.1).sleep();
	}

	std_msgs::String msg;
	stringstream ss;
	ss << killflag << "_";
	msg.data = ss.str();
	movement_publisher.publish(msg);

	ros::spinOnce();
	ros::spinOnce();

	ros::shutdown();

}

	bool loadImages()
	{
	

		image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_3.25_0.4_0.86_-0.5_0_.jpg");
		image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_3.25_0.4_0.49_-0.86_0_.jpg");
		image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_3.25_0.4_0_-1_0_.jpg");
		image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_3.25_0.4_1_0_0_.jpg");
		image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_3.25_0.4_-0.5_-0.86_0_.jpg");

		image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_2.75_0.4_0_-1_0_.jpg");
		image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_2.5_0.4_0.49_-0.86_0_.jpg");
		image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_-2.75_0.5_0.4_0.5_0.86_0_.jpg");
		image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_1.5_2.75_0.4_0.86_-0.5_0.jpg");
		image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.75_3.25_0.4_-0.86_0.5_0.jpg");


		for(int i = 0; i < image_names.size(); i++)
		{
			image_list.push_back(imread(image_names[i]));
		}

		return true;

        // string delimiter = "_";

        // // Go into folder and get all available photos and features.
        // const char * pstr = toPhotos.c_str();
        // fs::path p(pstr);
        // // Get a list of all filenames
        // GetAll(pstr, ret);

        // for(int i = 0; i < ret.size() && i < num_images; i++)
        // {
        // 	stringstream ss;
        // 	ss << toPhotos << ret[i].string();
        // 	image_names.push_back(ss.str());
        // 	if(i%100==0) std::cout << image_names[i] << endl;
        // 	image_list.push_back(imread(image_names[i]));
        // }

        // if(image_list.size() == 0)
        // 	cout << "failure!!" << endl;
        // return true;

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
		// string str = image_names[current_image];

		// std::vector<std::string> strs;
		// std::vector<float> vals;
		// str = str.substr(1,str.size()-1);
  //       boost::split(strs, str, boost::is_any_of("_"));

  //       // for(int i = 0; i < strs.size(); i++)
  //       // 	std::cout << "[ " << strs[i] << " ]" << endl;

  //       for(int i = 1; i < strs.size()-1; i++)
  //       {
  //       	vals.push_back(atof(strs[i].c_str()));
  //       	std::cout << vals[i-1] << ", ";
  //       }
  //       cout <<endl;

  //       float nangle = 0;
  //       nangle = atan2(vals[3], vals[4]) * 180.0 / 3.1415;

  //       float theta = (float) ((rand()%5)-10)*30;          // -60 60
		// float dist = (float) ((rand()%2)-4)*0.25;

		// theta += nangle;

		// int yy = theta/30;
		// theta = (int) yy*30;

		// vals[0] += cos(theta*3.14159/180.0)*dist;
		// vals[1] += sin(theta*3.14159/180.0)*dist;

		// int xx = vals[0]/.249;
		// vals[0] = round(xx*.25);
		// if(vals[0] == 0.0)
		// 	vals[0] = ((rand()%4)+1)*.25;

		// xx = vals[1]/.249;
		// vals[1] = round(xx*.25);

		// vals[3] = round(cos(theta*3.14159/180.0));
		// vals[4] = round(sin(theta*3.14159/180.0));

		// for(int i = 0; i < vals.size(); i++)
  //       {
  //       	std::cout << vals[i] << " ";
  //       }
  //       cout <<endl;

  //       std::vector<float> ret;
		// stringstream ss;
		// ss << toPhotos << "_" << vals[0] << "_" << vals[1] << "_" << vals[2] << "_" << round(vals[3]) << "_" << round(vals[4]) << "_" << round(vals[5]) << "_" << ".jpg";
		// std::cout << ss.str() << endl;

		// for(int i = 0; i < image_names.size(); i++)
		// {
		// 	if(!image_names[i].compare(ss.str()))
		// 	{
		// 		current_image = i;
		// 		cout << "image name from vector #" << i << "  " <<image_names[i] << endl;
		// 		ret.push_back(dist);
  //       		ret.push_back(theta);
  //       		return ret;
		// 	}
		// }
		vector<float> ret;
		ret.push_back(0);
		ret.push_back(0);



        // MCL::Perspective P(vals);

        return ret;


	}

    float round(float x)
	{
	    return (float) ((int) (x*100))/100.0;
	}