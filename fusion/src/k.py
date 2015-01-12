#! /usr/bin/env python

import roslib
import rospy
import actionlib
from test.msg import *
if __name__ == '__main__':
    rospy.init_node('do_dishes_client')
    client = actionlib.SimpleActionClient('fusion',test.msg.test_msgAction)
    print "will wait for server"
    client.wait_for_server()

    goal = test_msgGoal()
    goal.request="start"
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result()
    #a=client.get_state()
    if client.get_state()
    
