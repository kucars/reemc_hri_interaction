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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
 
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
using namespace ros;
using namespace cv;

#define PI 3.14159265

typedef actionlib::SimpleActionClient<test::test_msgAction> Client;


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");
  int a;
  // create the action client
  // true causes the client to spin its own thread
  Client ac("fusion", true);
  test::test_msgGoal goal;
  while(ros::ok())
  {
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  cout<<"what is teh command ?\n";
  cin>>a;
  if(a==1)
  	goal.request ="start";
  else if (a==2)
  	goal.request ="stop";
  else
  	break;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    cout<<"recieved: "<<ac.getResult()<<endl;
    
  }
  else
    ROS_INFO("Action did not finish before the time out.");
   }
  //exit
  return 0;
}
