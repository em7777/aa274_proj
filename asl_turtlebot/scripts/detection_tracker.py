#!/usr/bin/env python

import rospy
from asl_turtlebot.msg import DetectedObject, DetectedObjectList, TrackedObject, TrackedObjectList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
import numpy as np
import tf

STOP_SIGN_TH = 0.5
CONFIDENCE_TH = 0.75
FOOD_LIST = ["zebra", "donut", "banana", "traffic_light"]

# the main dictionary contains values that are objects of this class
class item():
    # def __init__(self, x, y, th, confidence):
    def __init__(self, x, y, th, pose, confidence):
        self.pose = pose
        self.x = x
        self.y = y
        self.th = th
        self.confidence = confidence

class detection_tracker():
    def __init__(self):
        rospy.init_node('detection_tracker', anonymous=True)
        self.detected_items = {}    #key = item name; value = item() object
        self.x = 0
        self.y = 0
        self.th = 0
        self.pose = None
        self.stop_counter = 0
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber('/detector/objects', DetectedObjectList, self.detection_callback)
        rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.pub = rospy.Publisher('/object_knowledge', TrackedObjectList, queue_size=5)
        self.pub_marker = rospy.Publisher('/object_marker', Marker, queue_size=5)
        while not self.pose:
            rospy.sleep(1)
        home = item(self.x, self.y, self.th, self.pose, 1.0)
        self.detected_items['home'] = home
        rospy.spin()

    def custom_print(self):
        print "^^^^^^^^^^^^^^^^^^^^^^^^"
        for key in self.detected_items:
            print "{} at ({},{},{}) measured at {}".format(key,
                self.detected_items[key].x,
                self.detected_items[key].y,
                self.detected_items[key].th,
                self.detected_items[key].confidence)

    def detection_callback(self, data):
        # for all detected objects, check if they need to be added to the dict
        for object in data.ob_msgs:
            # skip objects with less than 0.8 confidence
            if object.confidence < CONFIDENCE_TH and object.name != "stop_sign":
                continue
            # making an item() object
            # new_item = item(object_x, object_y, object_th, object.distance)
            new_item = item(self.x, self.y, self.th, self.pose, object.confidence)

            # if its a stop sign add it and increase counter
            if object.name == "stop_sign":
                # check if we have that stop_sign already
                already_detected = False
                detected_index = -1
                for i in range(self.stop_counter):
                    d = np.sqrt(np.square(new_item.x-self.detected_items["stop_sign"+str(i)].x) +
                                np.square(new_item.y-self.detected_items["stop_sign"+str(i)].y))
                    if d < STOP_SIGN_TH:
                        already_detected = True
                        detected_index = i
                        break
                # add the stop sign if it has not been detected
                if not already_detected:
                    self.detected_items[object.name+str(self.stop_counter)] = new_item
                    self.stop_counter += 1
                # else update the confidence if closer than before:
                elif object.confidence < self.detected_items["stop_sign"+str(detected_index)].confidence:
                    self.detected_items["stop_sign"+str(detected_index)].confidence = object.confidence

            # if the object is anything in the food list
            elif object.name in FOOD_LIST:
                # if new object, add it
                if object.name not in self.detected_items.keys():
                    self.detected_items[object.name] = new_item
                # if old object, but with better measurements, update it
                elif object.confidence < self.detected_items[object.name].confidence:
                    self.detected_items[object.name].confidence = object.confidence
        # FOR TESTING PURPOSES
        self.custom_print()

    # publish data everytime a new odom message comes out
    def write_dict_to_topic(self):
        if not self.detected_items:
            return
        msg_list = TrackedObjectList()
        for key in self.detected_items:
            msg = TrackedObject()
            msg.name = key
            msg.x = self.detected_items[key].x
            msg.y = self.detected_items[key].y
            msg.th = self.detected_items[key].th
            msg.robot_pose = self.detected_items[key].pose
            msg.confidence = self.detected_items[key].confidence
            msg_list.ob_msgs.append(msg)
        self.pub.publish(msg_list)
        # Publish Markers
        point_list = []
        color_list = []
        for key in self.detected_items:
            point_list.append(Point(self.detected_items[key].x,self.detected_items[key].y,0.5))
            if key[0:len(key)-1] == "stop_sign":
                color_list.append(ColorRGBA(1.0, 0, 0, 1.0))
            elif key == "home":
                color_list.append(ColorRGBA(0,0,1.0,1.0))
            elif key == "traffic_light":
                color_list.append(ColorRGBA(0,0,0,1.0))
            else:
                color_list.append(ColorRGBA(0, 1.0, 0.1, 1.0))
        m = Marker()
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.header.frame_id = "map"
        m.scale.x = 0.05
        m.scale.y = 0.05
        m.scale.z = 1.0
        m.points = point_list
        m.colors = color_list
        self.pub_marker.publish(m)

    # updates the pose of the robot everytime a new odom message comes out
    def update_pose(self, data):
        # transform to map coordinates
        self.tf_listener.waitForTransform("/map","/base_footprint",rospy.Time(),rospy.Duration(3.0))
        (translation,rotation) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        self.x = translation[0]
        self.y = translation[1]
        euler = tf.transformations.euler_from_quaternion(rotation)
        self.th = euler[2]
        # t = self.tf_listener.getLatestCommonTime("/map","/odom")
        # pose_to_transform = PoseStamped()
        # pose_to_transform.header = data.header
        # pose_to_transform.pose = data.pose.pose
        # pose_in_map = self.tf_listener.transformPose("/map", pose_to_transform)
        # # unpack
        # self.x = pose_in_map.pose.position.x
        # self.y = pose_in_map.pose.position.y
        # self.th = pose_in_map.pose.orientation.w
        self.pose = PoseWithCovariance()
        self.pose.pose.position.x = self.x
        self.pose.pose.position.y = self.y
        # self.pose.pose.orientation.z = self.th
        self.pose.pose.orientation = data.pose.pose.orientation
        self.write_dict_to_topic()


if __name__ == "__main__":
    try:
        tracker = detection_tracker()
    except rospy.ROSInterruptException:
        pass
