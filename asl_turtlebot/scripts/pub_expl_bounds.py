#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PolygonStamped
from frontier_exploration.srv import *
def expl_bounds():
    pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)
    rospy.init_node('expl_bounds', anonymous=True)
    rospy.loginfo("Publishing exploration boundaries")
    my_pt = PointStamped()
    my_pt.header.frame_id = "map"
    rospy.sleep(1)
    delay = 0.25


    # Point 1
    now = rospy.get_rostime()
    my_pt.header.stamp.secs = now.secs
    my_pt.header.stamp.nsecs = now.nsecs
    my_pt.point.x = 7
    my_pt.point.y = -3.2
    my_pt.point.z = 0.0
    pub.publish(my_pt)
    rospy.sleep(delay)

    # Point 2
    now = rospy.get_rostime()
    my_pt.header.stamp.secs = now.secs
    my_pt.header.stamp.nsecs = now.nsecs
    my_pt.point.x = 7
    my_pt.point.y = 6.4
    pub.publish(my_pt)
    rospy.sleep(delay)

    # # Point 3
    now = rospy.get_rostime()
    my_pt.header.stamp.secs = now.secs
    my_pt.header.stamp.nsecs = now.nsecs
    my_pt.point.x = -5
    my_pt.point.y = 6.4
    pub.publish(my_pt)
    rospy.sleep(delay)

    # # Point 4
    now = rospy.get_rostime()
    my_pt.header.stamp.secs = now.secs
    my_pt.header.stamp.nsecs = now.nsecs
    my_pt.point.x = -5
    my_pt.point.y = -3.2
    pub.publish(my_pt)
    rospy.sleep(delay)

    # # Point 5 (close polygon)
    now = rospy.get_rostime()
    my_pt.header.stamp.secs = now.secs
    my_pt.header.stamp.nsecs = now.nsecs
    my_pt.point.x = 5
    my_pt.point.y = 0
    my_pt.point.z = 0
    pub.publish(my_pt)
    rospy.sleep(delay)

    # # Point 6
    now = rospy.get_rostime()
    my_pt.header.stamp.secs = now.secs
    my_pt.header.stamp.nsecs = now.nsecs
    my_pt.point.x = 1
    my_pt.point.y = 1
    pub.publish(my_pt)
    rospy.sleep(delay)

# import rospy
# from geometry_msgs.msg import Point32
# from geometry_msgs.msg import PointStamped
# from geometry_msgs.msg import PolygonStamped
# from frontier_exploration.srv import *
# def expl_bounds():
#     pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)
#     rospy.init_node('expl_bounds', anonymous=True)
#     rospy.loginfo("Publishing exploration boundaries")
#     my_pt = PointStamped()
#     my_pt.header.frame_id = "map"
#     rospy.sleep(1)
#     delay = 0.25


#     # Point 1
#     now = rospy.get_rostime()
#     my_pt.header.stamp.secs = now.secs
#     my_pt.header.stamp.nsecs = now.nsecs
#     my_pt.point.x = 4
#     my_pt.point.y = 0
#     my_pt.point.z = 0.0
#     pub.publish(my_pt)
#     rospy.sleep(delay)

#     # Point 2
#     now = rospy.get_rostime()
#     my_pt.header.stamp.secs = now.secs
#     my_pt.header.stamp.nsecs = now.nsecs
#     my_pt.point.x = 4
#     my_pt.point.y = 3.2
#     pub.publish(my_pt)
#     rospy.sleep(delay)

#     # # Point 3
#     now = rospy.get_rostime()
#     my_pt.header.stamp.secs = now.secs
#     my_pt.header.stamp.nsecs = now.nsecs
#     my_pt.point.x = -2
#     my_pt.point.y = 3.2
#     pub.publish(my_pt)
#     rospy.sleep(delay)

#     # # Point 4
#     now = rospy.get_rostime()
#     my_pt.header.stamp.secs = now.secs
#     my_pt.header.stamp.nsecs = now.nsecs
#     my_pt.point.x = -2
#     my_pt.point.y = 0
#     pub.publish(my_pt)
#     rospy.sleep(delay)

#     # # Point 5 (close polygon)
#     now = rospy.get_rostime()
#     my_pt.header.stamp.secs = now.secs
#     my_pt.header.stamp.nsecs = now.nsecs
#     my_pt.point.x = 4
#     my_pt.point.y = 0
#     my_pt.point.z = 0
#     pub.publish(my_pt)
#     rospy.sleep(delay)

#     # # Point 6
#     now = rospy.get_rostime()
#     my_pt.header.stamp.secs = now.secs
#     my_pt.header.stamp.nsecs = now.nsecs
#     my_pt.point.x = 1
#     my_pt.point.y = 1
#     pub.publish(my_pt)
#     rospy.sleep(delay)


if __name__ == '__main__':
    try:
        expl_bounds()
    except rospy.ROSInterruptException:
        pass
