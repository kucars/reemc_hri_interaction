#!/usr/bin/env python
# license removed for brevity
#python imports
import pylab
import sys
import subprocess
import math
#ROS imports
import rospy
import rospkg
from pal_interaction_msgs.msg import DirectionOfArrival

rospy.init_node('sound_Localization_node')
rospy.loginfo('starting sound Localization node')
pub = rospy.Publisher('sound_Localization', DirectionOfArrival, queue_size=10)
r = rospy.Rate(10) # 10hz

rospack = rospkg.RosPack()
hark_node= rospack.get_path('test')



def parse_source( source ):
	tmp=source.replace('<Vector<ObjectRef> <Source<x ','')
	source_id=tmp[tmp.rfind('id')+2:tmp.rfind('>')]
	power=tmp[tmp.rfind('power ')+5:tmp.rfind('id')-2]
	tmp=tmp[0:tmp.rfind('power')]
	tmp = ''.join(c for c in tmp if c in '1234567890- .')
	vals=tmp.split()
	val1=float(vals[0])
	if vals[1].count(".") <1:
		val2=0.0
		val3=0.0
	elif vals[1].count(".") ==1:
		if vals[1].startswith("00"):
			val2=0.0
			val3=float(vals[1])
		else:
			val2=float(vals[1])
			val3=0.0
	elif vals[1].count(".") >1:
		val2=float(vals[1][0:vals[1].rfind(".")-1])
		val3=float(vals[1][vals[1].rfind("."):])
		
	print 'loc',val1,val2,val3
	print "power",power
	print "id",source_id
	pub_source(source_id,power,val1,val2,val3)
	return

def pub_source(source_id,power,valx,valy,valz):
	msg=DirectionOfArrival()
	msg.doa=0.0
	msg.prob=0.0
	msg.power=float(power)
 	if valy < -0.08:
 		msg.metadoa="Right"
 		msg.doa=abs((valy/math.pi)*180)
 	elif valy > 0.08:
 		msg.metadoa="Left"
 		msg.doa=abs((valy/math.pi)*180)
 	else:
 		msg.metadoa="Front"
 		msg.doa=(valy/math.pi)*180
	rospy.loginfo(msg)
	pub.publish(msg)
	return


if __name__ == '__main__':
	args=[hark_node+'/src/demoOnline2.n','plughw:1', hark_node+'/config/kinect_loc.dat',hark_node+'/Localization.txt']
	p = subprocess.Popen(args,stdout=subprocess.PIPE)
	source=""
	for line in p.stdout:
		line=line.strip(' \r\n')
		if line.find('<Vector<ObjectRef> >')>-1 or line.lower().find('DEG') >-1 or line.lower().find('clipping')>-1 or line.find('<')<0 or line.find('>')<0:
				continue
		else:
			source+=line
			if source.endswith('>') and 'id' in source:
				try:
					parse_source(source);
				except rospy.ROSInterruptException: pass
				source=''
