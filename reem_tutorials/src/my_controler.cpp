// C++ standard headers
#include <exception>
#include <string>
#include <iostream>
// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/PointHeadAction.h>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>

// PAL headers
#include <pal_detection_msgs/FaceDetections.h>
#include <pal_detection_msgs/FaceDetection.h>

#include <trajectory_msgs/JointTrajectory.h>
using namespace std;
using namespace ros;
///
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> Client;
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;

PointHeadClientPtr pointHeadClient;

////
string cameraInfoTopic = "/stereo/left/camera_info";
string cameraFrame     = "/stereo_optical_frame";
cv::Mat cameraIntrinsics;
pal_detection_msgs::FaceDetection face;
int frequency=10;
bool intrinsicsReceived=false;
///
void ch_loc(int  a,int b,Publisher& client);
void createPointHeadClient(PointHeadClientPtr& actionClient);
void change_pose(double a,Publisher& reset);
// ROS callback function for topic containing intrinsic parameters of a camera
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

void get_faces(const pal_detection_msgs::FaceDetections msg)
{
   if(msg.faces.size()>0)
   {
   	face=msg.faces[0];
   	ROS_INFO_STREAM("face"<<face.x<<","<<face.y);
   }
}
int main(int argc, char** argv)
{ 
  // Init the ROS node
	ros::init(argc, argv, "my_look_to_point");
	ROS_INFO("Starting look_to_point application ...");
	NodeHandle nh;
	//Client client("head_controller/point_head_action", true);
	Publisher reset =nh.advertise<trajectory_msgs::JointTrajectory>("head_controller/command",1000);
	Rate rate(frequency);
	intrinsicsReceived=false;
	double a;
	if (!ros::Time::waitForValid(ros::WallDuration(5.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }
  
 ros::Subscriber cameraInfoSub = nh.subscribe(cameraInfoTopic, 1, getCameraIntrinsics);
  
  ROS_INFO("Waiting for camera intrinsics ... ");
  while (!intrinsicsReceived )
  {
    ros::spinOnce();
    rate.sleep();
  }
  cameraInfoSub.shutdown();
 // createPointHeadClient(pointHeadClient);
	while ( ros::ok() )
	{
		cout<<"enter a value ";
  		cin>>a;
  		change_pose(a,reset);
	rate.sleep();
	ros::spinOnce();
	}
  return 0;
}

void ch_loc(int  a,int b,Publisher& client)
{

 /*   
  double x = 0;//( a  - cameraIntrinsics.at<double>(0,2) )/ cameraIntrinsics.at<double>(0,0);
  double y = 0;//( b  - cameraIntrinsics.at<double>(1,2) )/ cameraIntrinsics.at<double>(1,1);
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
  client.sendGoalAndWait(goal);*/
  std::vector<double> goals;
  goals.push_back(0.0);
  goals.push_back(0.0);
  trajectory_msgs::JointTrajectory traj;
  traj.header.stamp=ros::Time::now() + ros::Duration(0.01);
  traj.joint_names.push_back("head_1_joint");
  traj.joint_names.push_back("head_2_joint");
  traj.points.resize(1);
  traj.points[0].positions = goals;
  traj.points[0].velocities.push_back(0);
  traj.points[0].velocities.push_back(0);
  traj.points[0].time_from_start = ros::Duration(0.1);
  client.publish(traj);
}
void change_pose(double a,Publisher& reset)
{
	std::vector<double> goals;
	goals.push_back(a);
	goals.push_back(0.0);
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

void createPointHeadClient(PointHeadClientPtr& actionClient)
{
  ROS_INFO("Creating action client to head controller ...");

  actionClient.reset( new PointHeadClient("/head_controller/point_head_action") );

  int iterations = 0, max_iterations = 3;
  // Wait for head controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the point_head_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createPointHeadClient: head controller action server not available");
}
