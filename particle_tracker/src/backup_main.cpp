/******** Author: Juan Reyes L.        *****
********* juan.reyes.lopez@gmail.com   *****
********* Based on Rob Hess pf tracker *****
********* November 2011 UTFSM Chile    ****/


// PAL headers
#include <pal_detection_msgs/FaceDetection.h>
#include <pal_detection_msgs/FaceDetections.h>

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

typedef actionlib::SimpleActionClient<nao_msgs::JointAnglesWithSpeedAction> Client;
typedef actionlib::SimpleActionServer<test::test_msgAction> Server;
#define PI 3.14159265

Recognizer reco;
vector<Rect> faces;
vector<Mat> new_class;

Mat frame;
Mat hsvImg;
MatND hist1;

Rect*  Face;

bool pInit;
bool selected;
bool showParticles;
bool add_class;

int partn;
int center_x;

float sd;
float scale_sd;
double videoFOV_X;
double prev_angle;
particle* pArr;
particle best;

string box_text;
void track(Client& client);
void recognize();
void ch_loc(double angle,Client& client);
void find_offset(double x, double& angle);
void get_frame(const sensor_msgs::Image& img)
{
	cv_bridge::CvImageConstPtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(img);
    cvImgPtr->image.copyTo(frame);
}


void get_face(const pal_detection_msgs::FaceDetection msg)
{
	//when recieving new face, reset headpose && recognize face
	if(msg.width<=0 || msg.height==0)
	{
		selected=false;
		return;
	}
	Face=new Rect(msg.x,msg.y,msg.width,msg.height);
	faces.clear();
	faces.push_back(Rect(msg.x,msg.y,msg.width,msg.height));
	recognize();
	selected=true;
}


void execute(const test::test_msgGoalConstPtr& goal, Server* as)
{
  //called based on requests
  if(goal->request=="reset")
  {
  	//stop current tracking
  	selected=false;
  }
  else
  as->setAborted();
  
 
}

int main(int argc, char **argv)
{
    init(argc, argv, "pftracking_node");
    namedWindow("tracking",cv::WINDOW_AUTOSIZE);
    NodeHandle n_sub1,n_sub2,nh;
    Subscriber sub1,sub2;
    Server server(nh, "tracker_server", boost::bind(&execute, _1, &server), false);
	cout<<"initiated"<<endl;
	string fn=package::getPath("particle_tracker") + "/config/inputs.csv";
	partn = 100;
	sd = SD;
	particle* pArr;
	particle best;
	scale_sd = SCALE_SD;
	pInit=false;
	add_class=false;
	selected=false;
	showParticles=false;
	int frequency=10;
	center_x==0;
	videoFOV_X=60.9;
	Client client("joint_angles_action", true);
    if(argc <5)
	{
		cout<<"missing arguments"<<endl;
		return -1;
	}
  	sub1 = n_sub1.subscribe(argv[1], 1000, get_frame); //faces
	sub2 = n_sub2.subscribe(argv[2], 1000, get_face);  //frames
	frequency=atoi(argv[3]);
	showParticles=atoi(argv[4]);
	server.start();
	if(argc>5)
		videoFOV_X=atof(argv[5]);
	if(!reco.prepare(fn))
        {cout<<"error preparing LBP recognizer, could not read"<<fn<<endl; return -1;}
	Rate rate(frequency);
	server.start();
	while ( ros::ok() )
	{
		spinOnce();
		rate.sleep();
		track(client);
		if(!frame.empty())
		    {
		    	if(center_x==0) //first time only
		    		center_x=frame.size().width/2.0;
		    	imshow("tracking",frame);
			waitKey(30);}
			else cout<<"frame is empty"<<endl;
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

void track(Client& client)
{
    double displacement,angle;
    if(frame.empty())
    {cout<<"can't track"<<endl;return;}
    if(!selected)
    {
    	cout<<"no face to track"<<endl;
    	ch_loc(0,client);
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
		displacement =center_x-( best_p.x+ (best_p.width/2) );
        	find_offset(displacement,angle);
        	if(angle!=-1000)
        	{
        		ch_loc(angle,client);
        	}
        	else
        	{
        		cout<<"Not sending anything, I am happy with current loc!\n";
        	}        
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
    cout<<"recognize called"<<endl;
    Mat gray;
    cvtColor(frame,gray,CV_BGR2GRAY);
    if(frame.empty() || faces.empty())
    	{cout<<"something is empty"<<endl;return;}
	reco.find_multiple(gray,faces,prediction,confidence);
	if(prediction[0] <0)
		{
			cout<<"face unrecognized, adding new face to database"<<endl;
			add_class=true;
		}
	else
		cout<<"pred: "<<prediction[0]<<" conf: "<<confidence[0]<<endl;
}

void ch_loc(double angle,Client& client)
{

    
    //using ActioLib
    	client.waitForServer();
	nao_msgs::JointAnglesWithSpeedGoal goal;
	goal.joint_angles.joint_names.push_back("HeadYaw");
	goal.joint_angles.speed=0.12;     
    goal.joint_angles.relative=0; 
    goal.joint_angles.joint_angles.push_back(angle);
	client.sendGoalAndWait(goal);
	prev_angle=client.getResult()->goal_position.position[0];
	cout<<"rotated to: "<<prev_angle;
}

void find_offset(double x, double& angle)
{
	double angle_offset;
	angle_offset = ((videoFOV_X/2.0)*x)/center_x; 	
	cout<<"Angle Offset is:"<<angle_offset<<endl;
	if(abs(angle_offset)<=5)
	{
		angle = -1000;
		cout<<"Offset is too small to act on it!\n";
		return;
	}
	angle=prev_angle+(angle_offset*PI/180.0);
	cout<<"New angle to move to:"<<angle<<endl;
	
} 
