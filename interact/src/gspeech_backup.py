#!/usr/bin/env python
#ros imports
import roslib
import rospy
import actionlib
from std_msgs.msg import String
from std_msgs.msg import Float32
from pal_detection_msgs.msg import FaceDetection
from test.msg import *
from nao_msgs.msg import *
#other imports
from responder import *
from random import randint
import shlex,subprocess,os
import json
import string
import wolframalpha
cmd1='sox -r 16000 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 1%'
cmd2='wget -q -U "Mozilla/5.0" --post-file recording.flac --header="Content-Type: audio/x-flac; rate=16000" -O - "https://www.google.com/speech-api/v2/recognize?xjerr=1&client=chromium&lang=en-US&maxresults=10&pfilter=0&xjerr=1&key=AIzaSyByL8siBe1B6NSBPkVwCLr10_xkRJHVxdg"'
app_id='PY4R64-Y6JJJRER96'
#global sub_once
global pubs
global client
is_interacting=False
def fusion_output(msg):
	global is_interacting
	if msg.confidence !=0:
		if is_interacting: #if is_interacting is true, no ignore new face
			return
		else:		   #if is_interacting is false, start new interaction
			is_interacting=True
	else:
		is_interacting=False
#pubc = rospy.Publisher('confidence', Float32)


def answer(res):
	if '\'' in res:
		res=res.replace('\'','')
	print " the question is",res
	db_res=bot.answer_local_db(res)
	print len(db_res)
	if len(db_res) >=1:
		choice =randint(0,len(db_res)-1)
		if "END_INTERACTION" in db_res[choice]:
			say("nice talking to you human !")
			if reset_interaction("tracker_server"):
				print "tracker_server reset sucsessfull"
			if reset_interaction("fusion_server"):
				print "fusion_server reset sucsessfull"
		else:
			say(db_res[choice])
	elif len(db_res) ==0:
		say("ah, let me think")
		say(bot.answer_wolframalpha(res))
def rec():
	args2 = shlex.split(cmd2)#'sox -r 16000 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 1%'	
	os.system(cmd1)	
	output,error = subprocess.Popen(args2,stdout = subprocess.PIPE, stderr= subprocess.PIPE).communicate()
	if not error and len(output)>10: ## >10 to make sure that "output" is not empty
		raw_result=output.rstrip().split("\n");  #remove white spaces from the string end && split lines
		for line in raw_result:
			text=json.loads(line)
			if not text['result']:
				continue
			#take the first transcript [0], the first trascript has the highest confidence value
			res=text['result'][0]['alternative'][0]['transcript']
			#stop interaction
			if "thank you for your help" in res:
				say("OK, nice talking to you, human")
				is_interacting=False
				reset_interaction("tracker_server") #stop the tracking
				rospy.sleep(1)
				reset_interaction("fusion_server") #to request new info
				return
			answer(str(res))

def reset_interaction(request_srv):
	resetting_client = actionlib.SimpleActionClient(request_srv,test.msg.test_msgAction)
	print "resetting the service",request_srv
	resetting_client.wait_for_server()
	print "sending ",request_srv," goal request"
	goal = test_msgGoal()
	goal.request="reset"
	resetting_client.send_goal(goal)
	resetting_client.wait_for_result(rospy.Duration.from_sec(5.0))
	if resetting_client.get_state() !=3:
		return False
	else:
		return True
	
def say(strng):
	client = actionlib.SimpleActionClient("speech_action",nao_msgs.msg.SpeechWithFeedbackAction)
	client.wait_for_server()
	goal = SpeechWithFeedbackGoal()
	goal.say=str(strng)
	client.send_goal(goal)
	client.wait_for_result()		
if __name__ == '__main__':
	global bot
	try:
		rospy.init_node('interaction_node')
		bot=AIbot("PY4R64-Y6JJJRER96")
		pubs = rospy.Publisher('speech', String)
		sub_once=rospy.Subscriber("pal_face/best_face", FaceDetection, fusion_output)
		if reset_interaction("tracker_server"):
			print "tracker_server reset sucsessfull"
		rospy.sleep(1)
		if reset_interaction("fusion_server"):
			print "fusion_server reset sucsessfull"
		while not rospy.is_shutdown():
			if is_interacting:
				rec()
			else:
				rospy.wait_for_message("pal_face/best_face", FaceDetection)
	except rospy.ROSInterruptException: pass
	except KeyboardInterrupt:
		sys.exit(1)
   
