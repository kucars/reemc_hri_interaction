// PAL headers
#include <pal_interaction_msgs/DirectionOfArrival.h>
#include <pal_detection_msgs/FaceDetections.h>

//NAO headers
#include <nao_msgs/JointAnglesWithSpeed.h>

// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>

// Boost headers
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//C++ headers
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <stdint.h>

using namespace std;
using namespace cv;
using namespace ros;
using namespace boost;
namespace enc = sensor_msgs::image_encodings;
#define PI 3.14159265
pal_interaction_msgs::DirectionOfArrival sound_loc;


void ch_loc(Publisher & pub);

/*void get_depth_frame(const sensor_msgs::Image& img)
{

	cv_bridge::CvImageConstPtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(img,enc::TYPE_16UC1);
    cvImgPtr->image.copyTo(depth_frame);
    flip(depth_frame,depth_frame,1);
}*/

void get_sound_loc(const pal_interaction_msgs::DirectionOfArrival msg)
{
	sound_loc=msg;
}

int main(int argc, char **argv)
{
	init(argc, argv, "debug_node");
	double val;
	NodeHandle n_sub1,n_pub;
	Subscriber sub1;
	string s;
	sub1 = n_sub1.subscribe("sound_localisation", 1000, get_sound_loc);
	Publisher pub =n_pub.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles",1000);
	int frequency=10;

	if(argc <2)
	{
		cout<<"missing arguments"<<endl;
		return -1;
	}
	else if ( argc > 1 )
  	{
		frequency=atoi(argv[1]);
	}
	Rate rate(frequency);
	
	while (ros::ok())
	{
		spinOnce();
    		rate.sleep();
    		ch_loc(pub);
	}

	return 0;
}

void ch_loc(Publisher & pub)
{
	nao_msgs::JointAnglesWithSpeed msg;
	double angle;
	msg.speed=0.3;
	msg.joint_names.push_back("HeadYaw");
	if(sound_loc.metadoa=="")
		return;
	if(sound_loc.power>30)
		{			
			if(sound_loc.doa <10 && sound_loc.doa >-10)
				angle=0;
			else
				angle=(sound_loc.doa *PI) / 180.0;
	
			msg.joint_angles.push_back(angle);
			cout<<"angle: "<<angle<<endl;
			cout<<msg<<endl;
			pub.publish(msg);
		}
}

string ToString(int number)
{
 stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}

