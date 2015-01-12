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


#include "Recognizer.h"

using namespace std;
using namespace cv;
using namespace ros;
void recognize();
Mat frame;
vector<Rect> faces;
Recognizer reco;
string publish_topic;
void get_frame(const sensor_msgs::Image& img)
{
	cv_bridge::CvImageConstPtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(img);
    cvImgPtr->image.copyTo(frame);
}

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
	init(argc, argv, "recognition_node");
	namedWindow("reco",cv::WINDOW_AUTOSIZE);
	NodeHandle n_sub1,n_sub2;
	Subscriber sub1,sub2;
	string fn=package::getPath("pal_face_detector_opencv") +
                                 "/config/inputs.csv";
    string test;
	int frequency=10;
	if(argc <6)
	{
		cout<<"missing arguments"<<endl;
		return -1;
	}
	else if ( argc > 1 )
  	{
  		sub1 = n_sub1.subscribe(argv[1], 1000, get_frame); //faces
		sub2 = n_sub2.subscribe(argv[2], 1000, get_faces);  //frames
		frequency=atoi(argv[3]);
		fn=argv[4];
		publish_topic=argv[5];
	}
	if(!reco.prepare(fn))
        {cout<<"error preparing LBP recognizer, could not read"<<fn<<endl; return -1;}
	Rate rate(frequency);
	while (ros::ok())
	{
        if(!frame.empty())
        {
            recognize();
            imshow("reco",frame);
            waitKey(30);
        }
		spinOnce();
    	rate.sleep();
	}

	return 0;
}
void recognize()
{
    NodeHandle n_pub;
    ros::Publisher pub = n_pub.advertise<pal_detection_msgs::FaceDetections>(publish_topic, 1000);
    //pal_detection_msgs::FaceDetections msg;
    pal_detection_msgs::FaceDetection  detection;
    vector<int> prediction;
    vector<double>confidence;
    Mat gray;
    cvtColor(frame,gray,CV_BGR2GRAY);
    if(frame.empty() || faces.empty())
    	return;
    reco.find_multiple(gray,faces,prediction,confidence);
     detection.x           = static_cast<int>(faces[0].x);
     detection.y           = static_cast<int>(faces[0].y);
     detection.width       = static_cast<int>(faces[0].width);
     detection.height      = static_cast<int>(faces[0].height);
     detection.eyesLocated = false;
     detection.leftEyeX    = 0;
     detection.leftEyeY    = 0;
     detection.rightEyeX   = 0;
     detection.rightEyeY   = 0;
     detection.name        = reco.ToString(prediction[0]);
     detection.confidence  = confidence[0];
     string box_text = format("Prediction = %d, , %f precent", prediction[0],confidence[0]);
     putText(frame, box_text, Point(faces[0].x-10, faces[0].y-10),FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
     rectangle(frame,faces[0],CV_RGB(0,255,0));
    /*if(!frame.empty() && !faces.empty())
    {
        reco.find_multiple(gray,faces,prediction,confidence);
        for(int i=0;i<faces.size();i++)
        {
        //publish the detection according to the original image size
        detection.x           = static_cast<int>(faces[i].x);
        detection.y           = static_cast<int>(faces[i].y);
        detection.width       = static_cast<int>(faces[i].width);
        detection.height      = static_cast<int>(faces[i].height);
        detection.eyesLocated = false;
        detection.leftEyeX    = 0;
        detection.leftEyeY    = 0;
        detection.rightEyeX   = 0;
        detection.rightEyeY   = 0;

        detection.name        = reco.ToString(prediction[i]);
        detection.confidence  = confidence[i];
        msg.faces.push_back(detection);
        string box_text = format("Prediction = %d, , %f precent", prediction[i],confidence[i]);
        putText(frame, box_text, Point(faces[i].x-10, faces[i].y-10),FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
        rectangle(frame,faces[i],CV_RGB(0,255,0));
        }
        pub.publish(msg);

    }
    */

}
