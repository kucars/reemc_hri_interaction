// PAL headers
#include <pal_interaction_msgs/DirectionOfArrival.h>
#include <pal_detection_msgs/FaceDetections.h>
#include <pal_detection_msgs/FaceDetection.h>

//NAO headers
#include <nao_msgs/JointAnglesWithSpeed.h>

// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>
#include <std_msgs/String.h>

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

bool interact;
vector<Rect> faces;
pal_detection_msgs::FaceDetection best_face;
pal_interaction_msgs::DirectionOfArrival sound_loc;
std_msgs::String speech_msg;
Mat frame;

void ch_loc(Publisher & pub, bool reset);
string ToString(int number);
string ToString(double number);

void get_faces(const pal_detection_msgs::FaceDetections msg)
{
    Rect temp_face;
    faces.clear();
    if(msg.faces.size()>0)
    BOOST_FOREACH(const pal_detection_msgs::FaceDetection& face, msg.faces)
    {
        temp_face.x=face.x;
        temp_face.y=face.y;
        temp_face.width=face.width;
        temp_face.height=face.height;
        faces.push_back(temp_face);
    }
}

void get_sound_loc(const pal_interaction_msgs::DirectionOfArrival msg)
{
	if(msg.power>30)
		{sound_loc=msg; interact=true;}
}
void get_frame(const sensor_msgs::Image& img)
{
	cv_bridge::CvImageConstPtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(img);
    cvImgPtr->image.copyTo(frame);
    flip(frame,frame,1);
}

int main(int argc, char **argv)
{
	init(argc, argv, "debug_node");
	NodeHandle n_sub,n_pub;
	Subscriber sub1,sub2,sub3;
	sub1 = n_sub.subscribe("sound_localisation", 1000, get_sound_loc);
	sub2 = n_sub.subscribe("pal_face/faces", 1000, get_faces);
	Publisher pub1 =n_pub.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles",1000);
	Publisher speak= n_pub.advertise<std_msgs::String>("speech",1000);
	Publisher face=n_pub.advertise<pal_detection_msgs::FaceDetection>("pal_face/best_face",1000);
	sub3 = n_sub.subscribe("nao_camera/image_raw", 1000, get_frame);
	int frequency=10;
	interact=false;
	Rate rate(frequency);
	namedWindow("frame");
	bool fst_iter=true;
	double x=0,xx=0;
	while (ros::ok())
	{
		spinOnce();
    		rate.sleep();
    		if(frame.empty())
    			continue;
    		if(sound_loc.metadoa!="")
    		{
    			x=(sound_loc.doa*320)/(30.45);
    			line(frame, Point (320+x,0), Point(320+x,frame.size().height) , Scalar(5122), 2);
    			putText(frame, "Angle is"+ToString(sound_loc.doa), Point (15,15), FONT_HERSHEY_COMPLEX, 1, Scalar(5122));
    		
    		}
    		imshow("frame",frame);
            waitKey(30);
    		/*ch_loc(pub1,false);
		if(fst_iter)
		{
			//speech_msg.data="decision node initiated";
    			speak.publish(speech_msg);
    			ch_loc(pub1,true);
    			fst_iter=false;
    			continue;
    		}
		
    		else if(interact)
    		{
    			rate.sleep();
    			ch_loc(pub1,false);
    			ros::Duration(1.2).sleep();
    			spinOnce();
    			if(faces.empty())
    				{
    					ch_loc(pub1,true);
    					continue;
    				}
    			else
    			{
    				
    				//speech_msg.data="human detected";
    				//speak.publish(speech_msg);
    				best_face.x=faces[0].x;
        			best_face.y=faces[0].y;
        			best_face.width=faces[0].width;
        			best_face.height=faces[0].height;
        			face.publish(best_face);
    				interact=false;
    				sub1.shutdown();
    		}	}*/

    			
	}

	return 0;
}

void ch_loc(Publisher & pub, bool reset=false)
{
	nao_msgs::JointAnglesWithSpeed msg;
	double angle;
	msg.speed=0.3;
	msg.joint_names.push_back("HeadYaw");
	if(sound_loc.metadoa=="")
		{cout<<"returned"<<endl;return;}
	if (reset==true)
	{
		msg.joint_angles.push_back(0.0);
		pub.publish(msg);
		cout<<"reset"<<endl;
		return;
	}
	if(sound_loc.doa <10 && sound_loc.doa >-10)
		angle=0;
	else
		angle=(sound_loc.doa *PI) / 180.0;
	
	msg.joint_angles.push_back(angle);
	cout<<"angle: "<<angle<<endl;
	cout<<msg<<endl;
	pub.publish(msg);
}

string ToString(int number)
{
 stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}
string ToString(double number)
{
 stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}

