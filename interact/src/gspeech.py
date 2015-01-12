#!/usr/bin/env python

#ros imports
import roslib
import rospy
from pal_detection_msgs.msg import FaceDetection
from test.msg import *

#other imports
from AIbot import *
from robot_interface import *
from random import randint
import shlex,subprocess,os
import json
import string
cmd1='sox -r 16000 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 1%'
cmd2='wget -q -U "Mozilla/5.0" --post-file recording.flac --header="Content-Type: audio/x-flac; rate=16000" -O - "https://www.google.com/speech-api/v2/recognize?xjerr=1&client=chromium&lang=en-US&maxresults=10&pfilter=0&xjerr=1&key=AIzaSyByL8siBe1B6NSBPkVwCLr10_xkRJHVxdg"'
app_id='PY4R64-Y6JJJRER96'

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
				bot.robot_controller.say("OK, nice talking to you, human")
				is_interacting=False
				if not bot.robot_controller.reset_interaction("tracker_server"):
					print "error, could not reset the tracker server"
					rospy.sleep(0.5)
				if not bot.robot_controller.reset_interaction("fusion_server"):
					print "error, could not reset the fusion server"
				return
			bot.answer(str(res))
	
if __name__ == '__main__':
	global bot
	try:
		rospy.init_node('interaction_node')
		bot=AIbot("PY4R64-Y6JJJRER96")
		sub_once=rospy.Subscriber("pal_face/best_face", FaceDetection, fusion_output)
		if not bot.robot_controller.reset_interaction("tracker_server"):
			print "error, could not reset the tracker server"
			rospy.sleep(0.5)
		if not bot.robot_controller.reset_interaction("fusion_server"):
			print "error, could not reset the fusion server"
		while not rospy.is_shutdown():
			if is_interacting:
				rec()
			else:
				rospy.wait_for_message("pal_face/best_face", FaceDetection)
	except rospy.ROSInterruptException: pass
	except KeyboardInterrupt:
		sys.exit(1)
   
