#!/usr/bin/env python
#ros imports
import roslib;
import rospy
import actionlib
from nao_msgs.msg import *

cmd1='sox -r 16000 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 1%'
cmd2='wget -q -U "Mozilla/5.0" --post-file recording.flac --header="Content-Type: audio/x-flac; rate=16000" -O - "https://www.google.com/speech-api/v2/recognize?xjerr=1&client=chromium&lang=en-US&maxresults=10&pfilter=0&xjerr=1&key=AIzaSyByL8siBe1B6NSBPkVwCLr10_xkRJHVxdg"'
app_id='PY4R64-Y6JJJRER96'
#global sub_once
global pubs
global cnt
global client

def say(strng):
	client = actionlib.SimpleActionClient("speech_action",nao_msgs.msg.SpeechWithFeedbackAction)
	client.wait_for_server()
	goal = SpeechWithFeedbackGoal()
	goal.say=strng
	client.send_goal(goal)
	client.wait_for_result()
	
	
	
if __name__ == '__main__':
	try:
		rospy.init_node('speech')
		print "going to say"
		say("the quick brown fox jupms over the lazy dog")
		print "finished"
	except rospy.ROSInterruptException: pass
	except KeyboardInterrupt:
		sys.exit(1)
   
