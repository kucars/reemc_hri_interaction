// PAL headers
#include <pal_interaction_msgs/DirectionOfArrival.h>
#include <pal_detection_msgs/FaceDetections.h>
#include <pal_detection_msgs/FaceDetection.h>
 
// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
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

#include <fusion/test_msgAction.h>
#include <fusion/test_msgActionGoal.h>
#include <fusion/test_msgActionResult.h>

using namespace std;
using namespace ros;
using namespace cv;

#define PI 3.14159265

typedef actionlib::SimpleActionServer<fusion::test_msgAction> Server;
enum sound_beam_loc { NO_SOUND_DETECTED, SOUND_DETECTED};
enum face_stat {WAITING_FOR_REQUEST, FACE_REQUESTED,FACE_SELECTED};
string ToString(double number);
void change_pose(double angle,Publisher& head_controller);
void cal_weight();

sound_beam_loc SndBm_stat;
face_stat FACE_STAT;
Mat frame,cameraIntrinsics;

double sound_db_threshold;
double sound_beam;

vector<pal_detection_msgs::FaceDetection> faces;

int cnt;  //to keep the sound beam for some time
int best_weight_index;

bool send_face;
bool intrinsicsReceived;
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
	if(msg.power>sound_db_threshold &&SndBm_stat==NO_SOUND_DETECTED)
		{
			ROS_INFO_STREAM("POWER: "<<msg.power);
			ROS_INFO_STREAM("doa: "<<msg.metadoa<<" "<<msg.doa);
			sound_loc=msg;
			sound_beam=cameraIntrinsics.at<double>(0, 2);
			cnt=1;
			SndBm_stat=SOUND_DETECTED;
		}
		//else {SndBm_stat=NO_SOUND_DETECTED;}
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

void getCameraIntrinsics(const sensor_msgs::CameraInfoConstPtr& msg)
{
  cameraIntrinsics = cv::Mat::zeros(3,3,CV_64F);

  cameraIntrinsics.at<double>(0, 0) = msg->K[0]; //fx
  cameraIntrinsics.at<double>(1, 1) = msg->K[4]; //fy
  cameraIntrinsics.at<double>(0, 2) = msg->K[2]; //cx
  cameraIntrinsics.at<double>(1, 2) = msg->K[5]; //cy
  cameraIntrinsics.at<double>(2, 2) = 1;
  ROS_INFO_STREAM("recieving "<<cameraIntrinsics.at<double>(0, 0)<<","<<cameraIntrinsics.at<double>(1, 1)<<","<<cameraIntrinsics.at<double>(0, 2)<<","<<cameraIntrinsics.at<double>(1, 2));
  ROS_INFO_STREAM("center x"<<cameraIntrinsics.at<double>(0, 2));
  intrinsicsReceived = true;
}


void execute(const fusion::test_msgGoalConstPtr& goal, Server* as)
{
  //called based on requests
  if(goal->request=="reset")
  {
  	FACE_STAT=FACE_REQUESTED;
  	faces.clear();
  	as->setSucceeded();
  }
  else
  as->setAborted();
  
 
}
int main(int argc, char **argv)
{
    init(argc, argv, "debug_node");
    namedWindow("Data-fusion",cv::WINDOW_AUTOSIZE);
    NodeHandle nh;
    Subscriber face_sub=nh.subscribe("pal_face/faces", 1000, get_faces);
    Subscriber sound_sub=nh.subscribe("sound_localisation", 1000, get_sound_loc);
    Subscriber frame_sub=nh.subscribe("nao_camera/image_raw", 1000, get_frame);
    Subscriber cameraInfoSub;
    
    Publisher pub =nh.advertise<pal_detection_msgs::FaceDetection>("pal_face/best_face",1000);
    Publisher head_controller =nh.advertise<trajectory_msgs::JointTrajectory>("head_controller/command",1000);
    
    Server server(nh, "fusion_server", boost::bind(&execute, _1, &server), false);
    Rate rate(15);
    best_weight_index=-1;
    sound_db_threshold=10;
	cnt=0;
    intrinsicsReceived=false;
    SndBm_stat=NO_SOUND_DETECTED;
    FACE_STAT=WAITING_FOR_REQUEST;
    if(argc<7)
    {
    	ROS_INFO_STREAM("missing arguments");
    	return -1;
    }
    else
    {
    	face_sub=nh.subscribe(argv[1], 1000, get_faces);
    	sound_sub=nh.subscribe(argv[2], 1000, get_sound_loc);
    	frame_sub=nh.subscribe(argv[3], 1000, get_frame);
    	cameraInfoSub = nh.subscribe(argv[4], 1, getCameraIntrinsics);
    	pub =nh.advertise<pal_detection_msgs::FaceDetection>(argv[5],1000);
    	rate=Rate(atoi(argv[6]));
    	if(argc>6)
   	{
    		sound_db_threshold=atof(argv[6]);
    		ROS_INFO_STREAM("new audio db threshold "<<sound_db_threshold);
    	}
    }

	while (!intrinsicsReceived )
	{
		ros::spinOnce();
		rate.sleep();
	}
	cameraInfoSub.shutdown();
    server.start();
    while (ros::ok())
    { 
     	rate.sleep();   
		spinOnce();   
		if (frame.empty())
       		continue;
		if(cnt!=0)
		{
			if(cnt==1 && FACE_STAT==FACE_REQUESTED)
				{
					change_pose((-1)*sound_loc.doa*PI/180,head_controller);
					cnt++;
					continue;
				}
			//draw the sound estimated location
			line(frame, Point (static_cast<int>(sound_beam),0), Point(static_cast<int>(sound_beam),frame.size().height) , Scalar(5122), 2);
			if(cnt<30)
				{ROS_INFO_STREAM("CNT: "<<cnt<<" sOUND BEAM: "<<SndBm_stat); cnt++;}
			else 
			{
				cnt=0;
				faces.clear();
				if(FACE_STAT==FACE_REQUESTED)
						change_pose(0,head_controller);
						SndBm_stat=NO_SOUND_DETECTED;
						continue;
			}
       	}
       	//spinOnce();
   		if(FACE_STAT==FACE_REQUESTED && SndBm_stat==SOUND_DETECTED)
   		{
   			cal_weight();
   			if(best_weight_index!=-1)
   			 {
   			 	pub.publish(faces[best_weight_index]);
   		  	 	FACE_STAT=FACE_SELECTED;
   		  	 	SndBm_stat=NO_SOUND_DETECTED;
   		  	 	best_weight_index=-1;
   		  	 }
   		}
       	imshow("Data-fusion",frame);
        waitKey(30);
    }
    return 0;
}

void cal_weight()
{
	if(frame.empty() || faces.empty())
	{
		//ROS_INFO_STREAM("no need to calculate");
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
    			sound_weight=exp(-0.000069444*pow((faces[i].x+faces[i].width/2)-sound_beam,2));
    		//use the pal_detection_msgs::FaceDetection confidence variable to hold the weight
    		faces[i].confidence=abs(face_weight*sound_weight);
    		putText(frame,"w: "+ToString(faces[i].confidence), Point (faces[i].x-10,faces[i].y-10), FONT_HERSHEY_COMPLEX, 1, Scalar(5122));
    		if(best_weight_index ==-1)
    		{
    			best_weight_index=i;
    		}
    		else if(faces[i].confidence > faces[best_weight_index].confidence)
    				best_weight_index=i;
    		}
    		if(faces[best_weight_index].confidence ==0.0)
    			best_weight_index=-1;
}

void change_pose(double angle,Publisher& head_controller)
{
	std::vector<double> goals;
	
	trajectory_msgs::JointTrajectory traj;
	traj.header.stamp=ros::Time::now() + ros::Duration(0.01);
	traj.joint_names.push_back("head_1_joint");
	traj.joint_names.push_back("head_2_joint");
	traj.points.resize(1);
	
	traj.points[0].velocities.push_back(0);
	traj.points[0].velocities.push_back(0);
	traj.points[0].time_from_start = ros::Duration(0.1);
	
	//roate in x-axis only
	goals.push_back(angle);
	goals.push_back(0.0);

	traj.points[0].positions = goals;
	head_controller.publish(traj);
	ros::Duration(0.3).sleep();
}

string ToString(double number)
{
 stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}
