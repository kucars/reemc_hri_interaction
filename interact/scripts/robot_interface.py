#!/usr/bin/env python

#ROS imports
import roslib
import rospy
import actionlib
from test.msg import *
from nao_msgs.msg import *

#other imports
from random import randint
import string


#global sub_once
global pubs
global client
class Robot_interface:
	def reset_interaction(self,request_srv):
		resetting_client = actionlib.SimpleActionClient(request_srv,test.msg.test_msgAction)
		print "resetting the service",request_srv
		resetting_client.wait_for_server()
		print "sending the goal request to ",request_srv
		goal = test_msgGoal()
		goal.request="reset"
		resetting_client.send_goal(goal)
		resetting_client.wait_for_result(rospy.Duration.from_sec(5.0))
		if resetting_client.get_state() !=3:
			return False
		else:
			return True
	
	def say(self,strng):
		client = actionlib.SimpleActionClient("speech_action",nao_msgs.msg.SpeechWithFeedbackAction)
		client.wait_for_server()
		goal = SpeechWithFeedbackGoal()
		goal.say=str(strng)
		client.send_goal(goal)
		client.wait_for_result()
		
		
	 
