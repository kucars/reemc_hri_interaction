// PAL headers
#include <pal_detection_msgs/FaceDetection.h>
#include <pal_detection_msgs/FaceDetections.h>
#include <trajectory_msgs/JointTrajectory.h>

//NAO headers
#include <nao_msgs/JointAnglesWithSpeed.h>
#include <nao_msgs/JointAnglesWithSpeedAction.h>

// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/PointHeadAction.h>
#include <sensor_msgs/CameraInfo.h>

// Boost headers
#include <boost/foreach.hpp>

// OpenCV headers
#include <opencv/cv.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//C++ headers
#include <iostream>
#include <string.h>

//other
#include "tracker.h"
#include "particle.h"
#include "Recognizer.h"

#include <test/test_msgAction.h>
#include <test/test_msgActionGoal.h>
#include <test/test_msgActionResult.h>
using namespace std;
using namespace cv;
using namespace ros;



typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> Client;
typedef actionlib::SimpleActionServer<test::test_msgAction> Server;
#define PI 3.14159265
enum Track_state {TRACK_FACE, NO_FACE, RESET_TRACKING};
Track_state trck_stat;
Recognizer reco;
vector<Rect> faces;
vector<Mat> new_class;
std::vector<double> goals;

Mat frame;
Mat hsvImg;
MatND hist1;
Mat cameraIntrinsics;
Rect*  Face;

bool pInit;
bool selected;
bool showParticles;
bool add_class;
bool intrinsicsReceived;

int partn;
int center_x;

float sd;
float scale_sd;
double videoFOV_X;
double prev_angle;
particle* pArr;
particle best;

string cameraInfoTopic = "/stereo/left/camera_info";
string cameraFrame     = "/stereo_optical_frame";
string box_text;

void track(Client& client, Publisher& reset);
void recognize();
void reset_pose(Publisher& reset);
void ch_loc(int  a,int b,Client& client);
void get_frame(const sensor_msgs::Image& img)
{
	cv_bridge::CvImageConstPtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(img);
    cvImgPtr->image.copyTo(frame);
}


void get_face(const pal_detection_msgs::FaceDetection msg)
{
	if(trck_stat==RESET_TRACKING)
		return;
	//when recieving new face, reset headpose && recognize face
	if(msg.width<=0 || msg.height==0)
	{
		selected=false;
		trck_stat=NO_FACE;
		return;
	}
	Face=new Rect(msg.x,msg.y,msg.width,msg.height);
	faces.clear();
	faces.push_back(Rect(msg.x,msg.y,msg.width,msg.height));
	recognize();
	selected=true;
	trck_stat=TRACK_FACE;
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
  intrinsicsReceived = true;
}

void execute(const test::test_msgGoalConstPtr& goal, Server* as)
{
  //called based on requests
  if(goal->request=="reset")
  {
		//stop current tracking
		selected=false;
		trck_stat=RESET_TRACKING;
		delete Face;
		as->setSucceeded();
  }
  else
  	as->setAborted();
  
 
}

int main(int argc, char **argv)
{
	init(argc, argv, "pftracking_node");
	
	namedWindow("tracking",cv::WINDOW_AUTOSIZE);
	NodeHandle nh;
	Subscriber cameraFrame,TrackedFaceSub,cameraInfoSub;
	Server server(nh, "tracker_server", boost::bind(&execute, _1, &server), false);
	Publisher reset;
	ROS_INFO_STREAM("node initiated");
	string fn=package::getPath("particle_tracker") + "/config/inputs.csv";
	partn = 100;
	sd = SD;
	particle* pArr;
	particle best;
	scale_sd = SCALE_SD;
	pInit=false;
	add_class=false;
	selected=false;
	trck_stat=NO_FACE;
	showParticles=false;
	intrinsicsReceived=false;
	int frequency=10;
	goals.push_back(0.0);
	goals.push_back(0.0);
    if(argc <5)
	{
		ROS_INFO_STREAM("missing arguments");
		return -1;
	}
  	cameraFrame = nh.subscribe(argv[1], 1000, get_frame); //faces
	TrackedFaceSub = nh.subscribe(argv[2], 1000, get_face);  //frames
	cameraInfoSub = nh.subscribe(cameraInfoTopic, 1, getCameraIntrinsics);
	//reset =nh.advertise<trajectory_msgs::JointTrajectory>("head_controller/command",1000);
	Client client("head_controller/point_head_action", true);
	reset =nh.advertise<trajectory_msgs::JointTrajectory>("head_controller/command",1000);
	frequency=atoi(argv[3]);
	showParticles=atoi(argv[4]);
	server.start();
	if(!reco.prepare(fn))
    {
        	ROS_INFO_STREAM("error preparing LBP recognizer, could not read"<<fn);
        	return -1;
    }
	Rate rate(frequency);
	 while (!intrinsicsReceived )
  	{
  		ros::spinOnce();
		rate.sleep();
  	}
  	reset_pose(reset);
  	cameraInfoSub.shutdown();
	server.start();
	while ( ros::ok() )
	{
		rate.sleep();
		spinOnce();
		track(client,reset);
		if(!frame.empty())
		    {
		    	imshow("tracking",frame);
				waitKey(30);
			}
		else 
		ROS_INFO_STREAM("frame is empty");
		if(add_class)
		{
			if(new_class.size()>=20)
			{
				reco.add_class(new_class);
				add_class=false;
				new_class.clear();
			}
		}
    }
	return 0;
}

void track(Client& client, Publisher& reset)
{
    double displacement,angle;
    if(frame.empty())
    {
    	ROS_INFO_STREAM("can't track, to cam frame capturted !");
    	return;
    }
    if(trck_stat==RESET_TRACKING)
    {
    	reset_pose(reset);
    	trck_stat=NO_FACE;
    	return;
    }
     if(trck_stat==NO_FACE)
    {
    	ROS_INFO_STREAM("no face to track track");
    	return;
    }
    cvtColor(frame,hsvImg,CV_BGR2HSV);
		//executed just one time, to initalize
		if(!pInit) {
			hist1 = getHistogramHSV(hsvImg(*Face));
			pArr = init_particles(Face,&hist1,1,partn);
			pInit = true;
		}
		//update using gaussian random number generator
		updateParticles(pArr,partn,hsvImg,hist1,sd,scale_sd);
		//select the best particle
		best = getBest(pArr,partn);
		//make copies of best particles and erase worsts
		pArr = resampleParticles(pArr,partn);
		Rect best_p=Rect(Point(best.x-best.width/2*best.scale,best.y-best.height/2*best.scale),
				 	Point(best.x + best.width/2*best.scale,best.y + best.height/2*best.scale));
		rectangle(frame,best_p,Scalar(0,0,255) );
        	ch_loc(best_p.x+ (best_p.width/2), best_p.y+ (best_p.height/2),client);        
		if(add_class)
		{
			new_class.push_back(frame(best_p));
		}
		//show all particles
		for(int p=0; p < partn && showParticles; p++) {
			best = pArr[p];
			rectangle(frame,Point(best.x-best.width/2*best.scale,best.y-best.height/2*best.scale),
				 	Point(best.x + best.width/2*best.scale,best.y + best.height/2*best.scale),
					Scalar(255,0,0) );
		}
}

void recognize()
{
    vector<int> prediction;
    vector<double>confidence;
    ROS_INFO_STREAM("recognize called");
    Mat gray;
    cvtColor(frame,gray,CV_BGR2GRAY);
    if(frame.empty())
	{
		ROS_INFO_STREAM("cannot recognize, frame is empty");
		return;
	}
	 if(faces.empty())
	{
		ROS_INFO_STREAM("erros: no face to be recognized");
		return;
	}
	reco.find_multiple(gray,faces,prediction,confidence);
	if(prediction[0] <0)
		{
			ROS_INFO_STREAM("face unrecognized, adding new face to database");
			add_class=true;
		}
	else
		ROS_INFO_STREAM("pred: "<<prediction[0]<<" conf: "<<confidence[0]);
}

void ch_loc(int  a,int b,Client& client)
{
  double x = ( a  - cameraIntrinsics.at<double>(0,2) )/ cameraIntrinsics.at<double>(0,0);
  double y = ( b  - cameraIntrinsics.at<double>(1,2) )/ cameraIntrinsics.at<double>(1,1);
  double Z = 1.0; //define an arbitrary distance
  geometry_msgs::PointStamped pointStamped;
  
  pointStamped.header.frame_id = cameraFrame;
  pointStamped.header.stamp    = ros::Time::now();


  pointStamped.point.x = x * Z;
  pointStamped.point.y = y * Z;
  pointStamped.point.z = Z;
  //build the action goal
  control_msgs::PointHeadGoal goal;
  //the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
  goal.pointing_frame = cameraFrame;
  goal.pointing_axis.x = 0.0;
  goal.pointing_axis.y = 0.0;
  goal.pointing_axis.z = 1.0;
  goal.min_duration = ros::Duration(0.5);
  goal.max_velocity = 1.0;
  goal.target = pointStamped;
  ROS_INFO_STREAM("sending goal");
  //pointHeadClient->sendGoalAndWait(goal);
  client.sendGoal(goal);
}

void reset_pose(Publisher& reset)
{
	trajectory_msgs::JointTrajectory traj;
	traj.header.stamp=ros::Time::now() + ros::Duration(0.01);
	traj.joint_names.push_back("head_1_joint");
	traj.joint_names.push_back("head_2_joint");
	traj.points.resize(1);
	traj.points[0].positions = goals;
	traj.points[0].velocities.push_back(0);
	traj.points[0].velocities.push_back(0);
	traj.points[0].time_from_start = ros::Duration(0.1);
	reset.publish(traj);
}
