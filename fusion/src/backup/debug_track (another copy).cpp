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
 
using namespace std;
using namespace cv;
using namespace ros;
using namespace boost;
namespace enc = sensor_msgs::image_encodings;
#define PI 3.14159265
string ToString(int number);
void ch_loc(Publisher & pub, double angle);
Mat frame;
vector<Rect> faces;
double prev_angle;
pal_interaction_msgs::DirectionOfArrival sound_loc;
 
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
 
 
int main(int argc, char **argv)
{
    init(argc, argv, "debug_node");
    namedWindow("frame",cv::WINDOW_AUTOSIZE);
    NodeHandle n_sub1,n_pub;
    Subscriber sub1=n_sub1.subscribe("pal_face/faces", 1000, get_faces);  //frames
    Publisher pub =n_pub.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles",1000);
    Rate rate(10);
    // First we define our variables and constants.
    double cogX, cogY;
    const double videoFOV_X = 60.9; // degrees
    const double videoFOV_Y = 47.6; // degrees
    double kX =videoFOV_X / (640);
    double kY =videoFOV_Y/ (480);
     
    double center_x=640/2.0;
    double center_y=480/2.0;
     
    double valx,valy;
    while (ros::ok())
    {
    	 spinOnce();
         rate.sleep();      
       // If the target is not visible, sleep for one frame.
       if (faces.empty())
       		continue;

 
        // Else track it!
        cogX =( faces[0].x+ (faces[0].width/2) ) -center_x;
        cout<<"rotate       ";
        cout<<"dx: "<<cogX<<endl;
        cogY = ( faces[0].y+ (faces[0].height/2) ) -center_x;
        valx = (kX * cogX) *PI /180;
        valx= roundf(valx *100)/100;
        cout<<"vx: "<<valx<<endl;
        //valy = kY * cogY;
        ch_loc(pub,valx *-1);
        faces.clear();
        ros::Duration(1.2).sleep();
        
    }
    return 0;
}
 
void ch_loc(Publisher & pub, double angle)
{
	if(abs(angle) <0.1)
		return;
    nao_msgs::JointAnglesWithSpeed msg;
    msg.joint_names.push_back("HeadYaw");
    msg.speed=0.2;      
    msg.joint_angles.push_back(angle);
    pub.publish(msg);
    prev_angle=angle;
}
 
string ToString(int number)
{
 stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}
