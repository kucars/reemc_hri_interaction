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
int scale=2;
void draw_biggest_face(pal_detection_msgs::FaceDetection face);
bool same_face(pal_detection_msgs::FaceDetection face);
void track_face(cv::Mat& im,cv::Mat& tpl);

Mat frame,face_tpl;
Rect old_face;
void get_frame(const sensor_msgs::Image& img)
{
	cv_bridge::CvImageConstPtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(img);
    cvImgPtr->image.copyTo(frame);
	track_face(frame,face_tpl);
}

void get_face(const pal_detection_msgs::FaceDetection msg)
{
	old_face.x=msg.x;
	old_face.y=msg.y;
	old_face.width=msg.width;
	old_face.height=msg.height;
	frame.copyTo(face_tpl);
	face_tpl=face_tpl(old_face);
	draw_biggest_face(msg);
}
int main(int argc, char **argv)
{
  char a;
  
  init(argc, argv, "tracking_node"); 
  namedWindow("track",cv::WINDOW_AUTOSIZE);
  namedWindow("face_tpl",cv::WINDOW_AUTOSIZE);
  NodeHandle n_sub1,n_sub2;
  Subscriber sub1 = n_sub1.subscribe("/v4l/camera/image_raw", 1000, get_frame);
  Subscriber sub2 = n_sub2.subscribe("/pal_face/best_face", 1000, get_face);
  Rate rate(40);
   while ( ros::ok() )
  {a=0;
    if(!face_tpl.empty())
	{
		imshow( "face_tpl", face_tpl);
		a= cv::waitKey(10);
		/*
		if(a=='d')
		{
			cout<<"OK, saving .... \n";
			vector<int> params;
			params.push_back(CV_IMWRITE_PXM_BINARY);
			imwrite("test.pgm",face_tpl,params);
		}*/
	}
   	if(!frame.empty())
   		{imshow( "track", frame); cv::waitKey(1);}
    spinOnce();
    rate.sleep();
  }
  return 0;
}

void draw_biggest_face(pal_detection_msgs::FaceDetection face)
{
	Point p1=cv::Point(face.x,face.y);
	Point p2=cv::Point(face.x+face.width,face.y+face.width);
	rectangle(frame, p1,p2, 1234,2,8,0);
}
void track_face(cv::Mat& im,cv::Mat& tpl)
{
	if(tpl.empty() || im.empty())
		return;
	Mat dst,im_scaled,tpl_scaled;
	resize(im, im_scaled, Size(im.size().width/scale,im.size().height/scale));
	resize(tpl, tpl_scaled, Size(tpl.size().width/scale,tpl.size().height/scale));
	matchTemplate(im_scaled, tpl_scaled, dst, CV_TM_SQDIFF);
	normalize( dst, dst, 0, 1, cv::NORM_MINMAX, -1, Mat() );
	double minVal; double maxVal; Point minLoc; Point maxLoc;
	Point matchLoc;
	minMaxLoc( dst, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
	matchLoc.x = minLoc.x *scale;
	matchLoc.y = minLoc.y *scale;
	rectangle( im, matchLoc, Point( matchLoc.x + tpl.cols , matchLoc.y + tpl.rows ), Scalar::all(0), 2, 8, 0 );
}
