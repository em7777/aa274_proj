#!/usr/bin/env python

import rospy
from asl_turtlebot.msg import DetectedObject, DetectedObjectList, TrackedObject, TrackedObjectList
from nav_msgs.msg import Odometry
import numpy as np

STOP_SIGN_TH = 0.05

# the main dictionary contains values that are objects of this class
class item():
    def __init__(self, x, y, th, distance):
        self.x = x
        self.y = y
        self.th = th
        self.distance = distance

class detection_tracker():
    def __init__(self):
        rospy.init_node('detection_tracker', anonymous=True)
        self.detected_items = {}    #key = item name; value = item() object
        self.x = 0
        self.y = 0
        self.th = 0
        self.stop_counter = 0
        rospy.Subscriber('/detector/objects', DetectedObjectList, self.detection_callback)
        rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.pub = rospy.Publisher('/object_knowledge', TrackedObjectList, queue_size=5)
        rospy.spin()

    def custom_print(self):
        print "^^^^^^^^^^^^^^^^^^^^^^^^"
        for key in self.detected_items:
            print "{} at ({},{},{}) measured at {}".format(key,
                self.detected_items[key].x,
                self.detected_items[key].y,
                self.detected_items[key].th,
                self.detected_items[key].distance)

    def detection_callback(self, data):
        # for all detected objects, check if they need to be added to the dict
        for object in data.ob_msgs:
            # angle between world x axis and the line between robot and object
            alpha = 0.5*(object.thetaleft+object.thetaright) + self.th
            object_x = self.x + object.distance*np.cos(alpha)
            object_y = self.y + object.distance*np.sin(alpha)
            object_th = 0 # TODO: find object heading
            # making an item() object
            new_item = item(object_x, object_y, object_th, object.distance)

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
                # else update the distance if closer than before:
                elif object.distance < self.detected_items["stop_sign"+str(detected_index)].distance:
                    self.detected_items["stop_sign"+str(detected_index)].distance = object.distance

            # if the object is anything other than a stop sign
            else:
                # if new object, add it
                if object.name not in self.detected_items.keys():
                    self.detected_items[object.name] = new_item
                # if old object, but with better measurements, update it
                elif object.distance < self.detected_items[object.name].distance:
                    self.detected_items[object.name].distance = object.distance
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
            msg.distance = self.detected_items[key].distance
            msg_list.ob_msgs.append(msg)
        self.pub.publish(msg_list)


    # updates the pose of the robot everytime a new odom message comes out
    def update_pose(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.th = data.pose.pose.orientation.w
        self.write_dict_to_topic()


if __name__ == "__main__":
    try:
        tracker = detection_tracker()
    except rospy.ROSInterruptException:
        pass
