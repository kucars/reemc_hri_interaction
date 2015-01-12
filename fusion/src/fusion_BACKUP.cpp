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
#include <actionlib/server/simple_action_server.h>
 
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

#include <test/test_msgAction.h>
#include <test/test_msgActionGoal.h>
#include <test/test_msgActionResult.h>

using namespace std;
using namespace ros;
using namespace cv;

#define PI 3.14159265

typedef actionlib::SimpleActionServer<test::test_msgAction> Server;

string ToString(double number);
void cal_weight();


Mat frame;
double center_x=320;
double sound_beam;
vector<pal_detection_msgs::FaceDetection> faces;
int cnt;  //to keep the sound beam for some time
int best_weight_index;
bool send_face;
pal_interaction_msgs::DirectionOfArrival sound_loc;

void get_faces(const pal_detection_msgs::FaceDetections msg)
{
    faces.clear();
    if(msg.faces.size()>0)
    BOOST_FOREACH(const pal_detection_msgs::FaceDetection& face, msg.faces)
    {
        faces.push_back(face);
    }
}

void get_sound_loc(const pal_interaction_msgs::DirectionOfArrival msg)
{
	if(msg.power>10)
		{
			ROS_INFO_STREAM("doa: "<<msg.metadoa<<" "<<msg.doa);
			if(abs(msg.doa)<33)
			{sound_loc=msg;
			//multiplied by *-1 because the frame is nit flipped
			sound_beam=(-1* (msg.doa*center_x)/(30.45))+center_x;
			ROS_INFO_STREAM("sound_beam loc: "<<sound_beam);
			cnt=1;}
			else
			{
				ROS_INFO_STREAM("out of the FOV");
				sound_beam=-9000;
			}
		}
		else sound_beam=-9000;
}

void get_frame(const sensor_msgs::Image& img)
{
	cv_bridge::CvImageConstPtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(img);
    cvImgPtr->image.copyTo(frame);
   // flip(frame,frame,1);
    Rect face_rect;
	BOOST_FOREACH(pal_detection_msgs::FaceDetection face, faces)
	{
		face_rect.x=face.x;
		face_rect.y=face.y;
		face_rect.height=face.height;
		face_rect.width=face.width;
		rectangle(frame, face_rect,Scalar(84334));
	}
}

void execute(const test::test_msgGoalConstPtr& goal, Server* as)
{
  //called based on requests
  if(goal->request=="reset")
  {
  	send_face=true;
  	faces.clear();
  	as->setSucceeded();
  }
  else
  as->setAborted();
  
 
}
int main(int argc, char **argv)
{
    init(argc, argv, "debug_node");
    namedWindow("data-fusion",cv::WINDOW_AUTOSIZE);
    NodeHandle n_sub,n_pub,n;
    Subscriber face_sub=n_sub.subscribe("pal_face/faces", 1000, get_faces);
    Subscriber sound_sub=n_sub.subscribe("sound_localisation", 1000, get_sound_loc);
    Subscriber frame_sub=n_sub.subscribe("nao_camera/image_raw", 1000, get_frame);
    Publisher pub =n_pub.advertise<pal_detection_msgs::FaceDetection>("pal_face/best_face",1000);
    
    
    Server server(n, "fusion_server", boost::bind(&execute, _1, &server), false);
    Rate rate(15);
    best_weight_index=-1;
    send_face=false;
    sound_beam=-9000;
    if(argc<6)
    {
    	ROS_INFO_STREAM("missing arguments");
    	return -1;
    }
    else
    {
    	face_sub=n_sub.subscribe(argv[1], 1000, get_faces);
    	sound_sub=n_sub.subscribe(argv[2], 1000, get_sound_loc);
    	frame_sub=n_sub.subscribe(argv[3], 1000, get_frame);
    	pub =n_pub.advertise<pal_detection_msgs::FaceDetection>(argv[4],1000);
    	rate=Rate(atoi(argv[5]));
    }
    cnt=0;
    server.start();
    while (ros::ok())
    {
    	 spinOnce();
         rate.sleep();      
       if (frame.empty())
       		continue;
       		if(cnt!=0)
       		{
       			//draw the sound estimated location
       			line(frame, Point (static_cast<int>(sound_beam),0), Point(static_cast<int>(sound_beam),frame.size().height) , Scalar(5122), 2);
       			if(cnt<30)
       				cnt++;
       			else {cnt=0; sound_beam=-9000;}
       		}
   		cal_weight();
   		if(send_face)
   		{
   			if(best_weight_index!=-1)
   			 {
   			 	pub.publish(faces[best_weight_index]);
   		  	 	send_face=false;
   		  	 }
   		}
       	imshow("Ddata-fusion",frame);
        waitKey(30);
    }
    return 0;
}

void cal_weight()
{
	if(frame.empty() || faces.empty())
	{
		ROS_INFO_STREAM("no need to calculate");
		return;
	}
	//calculating faces weight
	double size_sum;
	//calculating the size sum
	BOOST_FOREACH(pal_detection_msgs::FaceDetection face, faces)
    	{
    		 size_sum+=(face.width*face.height);  		
    	}
    //assigning probabilities based on faces sizes
    double sound_weight=0,face_weight=0;
    		//BOOST_FOREACH(pal_detection_msgs::FaceDetection& face, faces)
    		for(int i=0;i<faces.size();i++)
    		{
    			face_weight =(faces[i].width*faces[i].height)/size_sum;
    			if(sound_beam!=-9000)
    			{
    				sound_weight=exp(-0.0001*pow((faces[i].x+faces[i].width/2)-sound_beam,2));
    			}
    			else {sound_weight=0;}
    		//ROS_INFO_STREAM("sound_weight is"<<sound_weight);
    		//use the pal_detection_msgs::FaceDetection confidence variable to hold the weight
    		faces[i].confidence=abs(face_weight*sound_weight);
    		putText(frame,"w: "+ToString(faces[i].confidence), Point (faces[i].x-10,faces[i].y-10), FONT_HERSHEY_COMPLEX, 1, Scalar(5122));
    		if(best_weight_index ==-1)
    			best_weight_index=i;
    		else if(faces[i].confidence > faces[best_weight_index].confidence)
    				best_weight_index=i;
    		}
    		if(faces[best_weight_index].confidence ==0.0)
    			best_weight_index=-1;
}
string ToString(double number)
{
 stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}
