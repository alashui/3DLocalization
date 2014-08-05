/**
*   @File   - ROSTEST/main.cpp
*   @Author - John Allard and Alex Rich, Summer 2014.
*   @Info   - This file was written at Harvey Mudd College under funcding from the NSF for the 2014 Summer Computer Science REU.
*             The purpose of this file is to be a quick test program for the 3DLocalization program. This file creates a virtual robot
*             that will traverse a virtual model of an environment while sending images rendered from its location in that environment
*             to the 3DLocalization for it to try and determine where the virtual robot is.
*             This program is a good example of how someone should design there robot control code to interface with our localization program.
*
**/

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

string dirName = "BirchLab/";//2ndFloorSprague/";
string pathToData = "../../../Data";
string toFeatures;// = pathToData + "/FeatureData/" + dirName;
string toPhotos;// = pathToData + "/RenderedImages/" + dirName;

const int handshake = 15;
const int readymove = 25;
const int guessdata = 35;
const int killflag = 666;
const int starting_move = 10;
const int finished_move = 20;

const int num_images = 2000;

float moves[16][2];


vector<Mat> image_list;
vector<string> image_names;
int current_image = 0;
Mat BestGuessImage = imread("../../../Data/RenderedImages/2ndFloorSprague/_0.5_3.25_0.4_0_-1_0_.jpg");
// Mat BestGuessImage;// = imread("../inputimages/square1/1.JPG");
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

    // Publish 'Start Move' command (10);
    msg.data = "10";
    movement_publisher.publish(msg);

    // Wait for the 3DL program to get the message and start waiting for the done moving command
    ros::spinOnce();
    ros::Duration(1).sleep();

    // Publish the finished moving command and the movement coordinates.
    ss << "20" << " " << temp[0] << " " << temp[1] << " ";
    std::cout << ss.str() << std::endl;
    msg.data = ss.str();
    movement_publisher.publish(msg);

    // if(current_image == image_list.size())
    //     current_image = 0;
    // // imshow("Robot Image", image_list[current_image]);

}

// Recieves data from the 3DL program such as the 'HandShake' command, 'ReadyMove' command, and meta data about the hypothosis of
// the robots location.
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
        std::cout << ss.str() << std::endl;
        BestGuessImage = imread(ss.str());

        if(BestGuessImage.empty())
        {
            cout << "image data corrupted in guess data callback" << endl;
            BestGuessImage = image_list[0];

            return;
        }
    }
}


int main(int argc, char **argv)
{
    dirName = argv[1];
    toFeatures = pathToData + "/FeatureData/" + dirName;
    toPhotos = pathToData + "/RenderedImages/" + dirName;
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

        // std::cout << "hererer" << std::endl;
        imshow("Robot Image", image_list[current_image]);
        imshow("Top Match", BestGuessImage);

        key = cv::waitKey(2);
        // if(key == ' ')
        //     current_image++;
        // if(current_image == image_list.size())
        //     current_image = 0;
        //ros::Duration(0.1).sleep();
    }

    std_msgs::String msg;
    stringstream ss;
    ss << killflag << "_";
    msg.data = ss.str();
    movement_publisher.publish(msg);

    ros::spinOnce();
    ros::spinOnce();

    ros::Duration(0.1).sleep();
    ros::spinOnce();
    ros::shutdown();

}

    bool loadImages()
    {
    image_names.clear();
    image_list.clear();

        // if(dirName == "BirchLab/")
        // // {
        //     image_names.push_back("../../../Data/RenderedImages/BirchLab/_-0.5_-0.4_1.5_-0.5_0.86_0_.jpg");
        //     image_names.push_back("../../../Data/RenderedImages/BirchLab/_1_0.6_1.5_-0.5_0.5_0_.jpg");
        //     image_names.push_back("../../../Data/RenderedImages/BirchLab/_1_0.6_1.5_0.5_-0.5_0_.jpg");
        //     image_names.push_back("../../../Data/RenderedImages/BirchLab/_1.5_5.6_1.5_-0.5_0.86_0_.jpg");
        //     image_names.push_back("../../../Data/RenderedImages/BirchLab/_1.5_5.6_1.5_0.5_-0.86_0_.jpg");
        //     image_names.push_back("../../../Data/RenderedImages/BirchLab/_1.5_5.6_1.5_-0.5_-0.86_0_.jpg ");
        //     image_names.push_back("../../../Data/RenderedImages/BirchLab/_1.5_5.6_1.5_0.5_0.86_0_.jpg");
        // }

        //     for(int i = 1; i <= 16; i++)
        //     {
        //         std::stringstream ss;
        //         ss << "../inputimages/square1/" << i << ".JPG";
        //         image_names.push_back(ss.str());
        //     }

        //     for(int i = 0; i < 3; i++)
        //     {
        //         moves[i][0] = 1;
        //         moves[i][1] = 0;
        //     }

        //     moves[3][0] = 0;
        //     moves[3][1] = 90;

        //     for(int i = 4; i < 7; i++)
        //     {
        //         moves[i][0] = 1;
        //         moves[i][1] = 0;
        //     }

        //     moves[7][0] = 0;
        //     moves[7][1] = 90;

        //     for(int i = 8; i < 11; i++)
        //     {
        //         moves[i][0] = 1;
        //         moves[i][1] = 0;
        //     }

        //     moves[11][0] = 0;
        //     moves[11][1] = 90;

        //     for(int i = 12; i < 15; i++)
        //     {
        //         moves[i][0] = 1;
        //         moves[i][1] = 0;
        //     }

        //     moves[15][0] = 0;
        //     moves[15][1] = 90;


        // Size size(804, 600);
        // for(int i = 0; i < image_names.size(); i++)
        // {
        //     Mat temp = imread(image_names[i]);
        //     Mat dest;
        //     resize(temp, dest, size);
        //     image_list.push_back(dest);
        // }

        // return true;





        // else
        //     {
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_3.25_0.4_0.86_-0.5_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_3.25_0.4_0.49_-0.86_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_3.25_0.4_0_-1_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_3.25_0.4_1_0_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_3.25_0.4_-0.5_-0.86_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_2.75_0.4_0_-1_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_2.5_0.4_0.49_-0.86_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.5_8_0.4_-0.86_-0.49_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_0.25_4.5_0.4_0_1_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_1.5_5.25_0.4_0_1_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_1.5_8_0.4_-0.86_-0.49_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_1.25_1.5_0.4_0.86_-0.5_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_1.25_1.75_0.4_-1_0_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_1.25_5.75_0.4_0.86_-0.5_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_1.75_3.75_0.4_0_-1_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_1.75_3.25_0.4_0.49_-0.86_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_1_2.75_0.4_-0.5_-0.86_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_1_8.25_0.4_0.86_0.49_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_2.5_0.25_0.4_-0.49_0.86_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_2.5_2.5_0.4_-0.86_0.5_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_-2.75_0.5_0.4_0.5_0.86_0_.jpg");
            image_names.push_back("../../../Data/RenderedImages/2ndFloorSprague/_1.5_2.75_0.4_0.86_-0.5_0.jpg");
        // }

        // Size size(804, 600);
        for(int i = 0; i < image_names.size(); i++)
        {
            // Mat temp = imread(image_names[i]);
            // Mat dest;
            // resize(temp, dest, size);
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
        //  stringstream ss;
        //  ss << toPhotos << ret[i].string();
        //  image_names.push_back(ss.str());
        //  if(i%100==0) std::cout << image_names[i] << endl;
        //  image_list.push_back(imread(image_names[i]));
        // }

        // if(image_list.size() == 0)
        //  cout << "failure!!" << endl;
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
        float trans = moves[current_image][0];
        float rotate = moves[current_image][1];

        current_image++;
        if(current_image == image_list.size())
            current_image = 0;

        vector<float> temp;
        temp.push_back(trans);
        temp.push_back(rotate);

        return temp;
    }

    float round(float x)
    {
        return (float) ((int) (x*100))/100.0;
    }