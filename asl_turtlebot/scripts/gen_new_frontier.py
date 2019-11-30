#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from frontier_exploration.srv import GetNextFrontier

def callback(msg):
	rate = rospy.Rate(0.25) #force new goal every 4s
	print msg.pose.pose
	pose = PoseStamped()
	now = rospy.get_rostime()
	pose.header.stamp.secs = now.secs
	pose.header.stamp.nsecs = now.nsecs
	pose.header.frame_id = "map"
	rospy.wait_for_service('/explore_server/explore_costmap/explore_boundary/get_next_frontier')
	try:
		gnf = rospy.ServiceProxy('/explore_server/explore_costmap/explore_boundary/get_next_frontier', GetNextFrontier)
		resp1 = gnf(pose)
		rospy.loginfo("publishing response")
		rospy.loginfo(resp1.next_frontier)
		rate.sleep()
		pub.publish(resp1.next_frontier)
	except rospy.ServiceException, e:
	    print "Service call failed: %s"%e

rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/odom', Odometry, callback)
pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
rospy.spin()