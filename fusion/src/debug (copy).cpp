// PAL headers
#include <pal_interaction_msgs/DirectionOfArrival.h>

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

using namespace std;
using namespace cv;
using namespace ros;
using namespace boost;

#define PI 3.14159265

void draw_loc();
Mat frame;
pal_interaction_msgs::DirectionOfArrival sound_loc;
RNG rng(12345);
void get_frame(const sensor_msgs::Image& img)
{
	cv_bridge::CvImageConstPtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(img);
    cvImgPtr->image.copyTo(frame);
    flip(frame,frame,1);
    line(frame, Point (frame.size().width/2,0), Point(frame.size().width/2,frame.size().height) , Scalar(5122), 2);
    line(frame, Point (0,frame.size().height/2), Point(frame.size().width,frame.size().height/2) , Scalar(5122), 2);
}
void get_sound_loc(const pal_interaction_msgs::DirectionOfArrival msg)
{
	sound_loc=msg;
}

int main(int argc, char **argv)
{
	init(argc, argv, "debug_node");
	namedWindow("frame",cv::WINDOW_AUTOSIZE);
	NodeHandle n_sub1,n_sub2;
	Subscriber sub1,sub2;
	sub2 = n_sub2.subscribe("sound_localisation", 1000, get_sound_loc); //faces
	int frequency=10;
	if(argc <3)
	{
		cout<<"missing arguments"<<endl;
		return -1;
	}
	else if ( argc > 1 )
  	{
  		sub1 = n_sub1.subscribe(argv[1], 1000, get_frame); //faces
		frequency=atoi(argv[2]);
	}
	Rate rate(frequency);
	while (ros::ok())
	{
        if(!frame.empty())
        {
        	draw_loc();
            imshow("frame",frame);
            waitKey(30);
        }
		spinOnce();
    	rate.sleep();
	}

	return 0;
}

void draw_loc()
{
	Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	if(sound_loc.metadoa=="")
		return;
	double distance=1/tan (90- sound_loc.doa * PI / 180.0 );
	if(sound_loc.metadoa=="Left")
		{
			rectangle(frame, Point (0,0), Point(frame.size().width/2,frame.size().height) , color, 2);
		}
	else if(sound_loc.metadoa=="Right")
		{
		rectangle(frame,Point(frame.size().width/2,0),Point (frame.size().width,frame.size().height), color, 2);
		}
}
