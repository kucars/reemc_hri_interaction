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
using namespace cv;
using namespace ros;
void draw_biggest_face(pal_detection_msgs::FaceDetection face);
bool same_face(pal_detection_msgs::FaceDetection face);
Mat frame,edited_frame;
pal_detection_msgs::FaceDetection old_face;
void get_frame(const sensor_msgs::Image& img)
{
cv_bridge::CvImageConstPtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(img);
    cvImgPtr->image.copyTo(frame);
    imshow("track",frame);
    cv::waitKey(5); 
}

void get_face(const pal_detection_msgs::FaceDetection msg)
{
	frame.copyTo(edited_frame);
	if(!same_face(msg))
		draw_biggest_face(msg);
}
int main(int argc, char **argv)
{
  int a;
  init(argc, argv, "track"); 
  namedWindow("track",cv::WINDOW_AUTOSIZE);
  NodeHandle n_sub1,n_sub2;
  Subscriber sub1 = n_sub1.subscribe("/v4l/camera/image_raw", 1000, get_frame);
  Subscriber sub2 = n_sub2.subscribe("/pal_face/best_face", 1000, get_face);
  Rate rate(500);
   while ( ros::ok() )
  {
    spinOnce();
    rate.sleep();
  }
  return 0;
}

void draw_biggest_face(pal_detection_msgs::FaceDetection face)
{
	Point p1=cv::Point(face.x,face.y);
	Point p2=cv::Point(face.x+face.width,face.y+face.width);
	rectangle(edited_frame, p1,p2, 1234,2,8,0);
	//imwrite("frame.png",frame);
	 if(!frame.empty())
    	{
    	imshow("track",edited_frame);
    	int c = cv::waitKey(5);
	if( (char)c == 'c' )  return; 
	}
    else cout<<"frame is empty \n";
}
bool same_face(pal_detection_msgs::FaceDetection face)
{
	if(old_face.x ==face.x && old_face.y==face.y && old_face.width==face.width)
		return true;
	else
		{ old_face=face;return false;}
	
}
