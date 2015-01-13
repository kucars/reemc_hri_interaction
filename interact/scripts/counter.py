#!/usr/bin/env python
import threading
import time
import rospy
import logging

class time_counter:
	def __init__(self,duration):
		self.count=0
		self.timeout=duration
		self.daemon_thread = threading.Thread(name='daemon', target=self.counter)
		self.daemon_thread.setDaemon(True)
		self.pause=False
		
		
	def counter(self):
		while True:
			if self.pause:
				continue
			else:
				if self.count<self.timeout:
					self.count+=1
					rospy.sleep(0.5)
					#keep counting
				else:
					self.count=0
					#end of timeout
				
	def start(self):
		count=0
		if not self.daemon_thread.isAlive():
			self.daemon_thread.start()
		else:
			self.pause=False
	def pause_thread(self):
		self.pause=True
	def get_counter(self):
		return self.count
	
