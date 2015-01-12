// PAL headers
#include <pal_detection_msgs/FaceDetection.h>
#include <pal_detection_msgs/FaceDetections.h>

// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>

// Boost headers
#include <boost/foreach.hpp>

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//C++ headers
#include <iostream>
using namespace std;
const void find_biggest_face(vector<pal_detection_msgs::FaceDetection> faces);

void chatterCallback(const pal_detection_msgs::FaceDetections msg)
{
	if(msg.faces.size()>0)
	find_biggest_face(msg.faces);
	else cout<<"no faces detected \n";
}


int main(int argc, char **argv)
{
  int a;
  ros::init(argc, argv, "decision_node");  
  ros::NodeHandle n_sub;
  ros::Subscriber sub = n_sub.subscribe("/pal_face/faces", 1000, chatterCallback);
  ros::NodeHandle n_pub;
  ros::Publisher pub = n_pub.advertise<pal_detection_msgs::FaceDetection>("/pal_face/best_face", 1000);
  ros::Rate rate(10);
   while ( ros::ok() )
  {
    ros::spinOnce();
    rate.sleep();    
  }
  return 0;
}
const void find_biggest_face(vector<pal_detection_msgs::FaceDetection> faces)
{
	pal_detection_msgs::FaceDetection biggest_face=faces[0];
	ros::NodeHandle n_pub;
	ros::Publisher pub = n_pub.advertise<pal_detection_msgs::FaceDetection>("/pal_face/best_face", 1000);
	BOOST_FOREACH(const pal_detection_msgs::FaceDetection& face, faces)
  	{
  		if((face.width*face.height)>(biggest_face.width*biggest_face.height))
  			{
			    biggest_face.x           = static_cast<int>(face.x);
			    biggest_face.y           = static_cast<int>(face.y);
			    biggest_face.width       = static_cast<int>(face.width);
			    biggest_face.height      = static_cast<int>(face.height);
    			}
  	}
    cout<<"sending ... \n";
    biggest_face.x           = static_cast<int>(biggest_face.x);
    biggest_face.y           = static_cast<int>(biggest_face.y);
    biggest_face.width       = static_cast<int>(biggest_face.width);
    biggest_face.height      = static_cast<int>(biggest_face.height);
    biggest_face.eyesLocated = false;
    biggest_face.leftEyeX    = 0;
    biggest_face.leftEyeY    = 0;
    biggest_face.rightEyeX   = 0;
    biggest_face.rightEyeY   = 0;

    biggest_face.name        = "";
    biggest_face.confidence  = 0;
    pub.publish(biggest_face);
 }
