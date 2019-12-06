#!/usr/bin/env python

import copy
import rospy
import smach
import smach_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from frontier_exploration.srv import GetNextFrontier
from move_base_msgs.msg import MoveBaseActionResult
import roslaunch
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from asl_turtlebot.msg import DetectedObject, DetectedObjectList, TrackedObject, TrackedObjectList
# # define state Discover

HOME = 'home'
orders = []


class Discover(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Discover')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.mbResultSub = rospy.Subscriber(
            '/move_base/result', MoveBaseActionResult, self.mbCallback)
        self.mbPub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)
        self.vendorLocationSub = rospy.Subscriber(
            '/object_knowledge', TrackedObjectList, self.vLcallback)
        self.mbResultSub = rospy.Subscriber(
            '/move_base/result', MoveBaseActionResult, self.mbCallback)
        self.poseLookup = None
        if len(orders) > 0 and orders[-1] != HOME:
            orders.append(HOME)
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.interrupted = False

    def vLcallback(self, msg):
        if not self.poseLookup:
            self.poseLookup = {}
            for obj in msg:
                self.poseLookup[obj.name] = obj.pose

    def mbCallback(self, msg):
        if (msg.status.status == 3):
            self.reachedGoal = True

    def execute(self, userdata):
        rospy.loginfo('Executing state Navigation')

        while len(orders) > 0:
            curObj = orders[0]
            self.reachedGoal = False
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose = self.poseLookup[curObj]
            self.client.send_goal(self.goal)
            wait = self.client.wait_for_result()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            while not self.reachedGoal and not self.interrupted:
                rospy.sleep(1)
            if self.interrupted:
                return 'failed'
            if self.reachedGoal:
                del orders[0]

        return 'success'

# define state Bar


class OHShit(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Executing state OHShit')
        return 'outcome2'

# define state Bar


class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Idle')
        return 'outcome2'


class ProcessOrder(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'fail'])
        # TODO: update topic name and message type
        self.orderSub = rospy.Subscriber(
            'ORDER_TOPIC', ORDER_MSG, self.orderCallback)
        self.orderPub = rospy.Publisher('ORDER_TOPIC', ORDER_MSG)
        # TODO: test if we need to cancel move base command early
        # self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odomCallback)
        self.mbResultSub = rospy.Subscriber(
            '/move_base/result', MoveBaseActionResult, self.mbCallback)
        self.mbPub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)

        self.reachedGoal = False
        self.orders = []
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        # TODO: better orientation
        self.goal.target_pose.pose.position.z = 0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = -0.670949053335
        self.goal.target_pose.pose.orientation.w = 0.741503450989
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def mbCallback(self, msg):
        if (msg.status.status == 3):
            self.reachedGoal = True

    def orderCallback(self, msg):
        self.orders = msg

    def execute(self, ud):
        while len(self.orders) == 0:
            pass
        for order in orders:
            self.reachedGoal = False
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose.position.x = order.x
            self.goal.target_pose.pose.position.y = order.y
            self.client.send_goal(self.goal)
            wait = self.client.wait_for_result()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            while not self.reachedGoal:
                rospy.sleep(1)
        self.orderPub.publish([])
        return 'succeeded'
    # def odomCallback(self, msg):
    #     # self.pose.pose = msg.pose.pose
    #     # #strangely, passing 0 starting pose as arg works...
    #     self.pose.header.frame_id = "map"
    #     now = rospy.get_rostime()
    #     self.pose.header.stamp.secs = now.secs
    #     self.pose.header.stamp.nsecs = now.nsecs


class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'fail'])
        self.sub = rospy.Subscriber('ORDER_TOPIC', ORDER_MSG, self.callback)
        self.stayIdle = True

    def callback(self, msg):
        if len(msg) > 0:
            self.stayIdle = False

    def execute(self, userdata):
        rospy.logging('Entering idle state')
        while self.stayIdle:
            pass
        return 'succeeded'

# main


def main():
    rospy.init_node('aa274_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Complete'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Discover', Discover(),
                               transitions={'success': 'Navigation'})

        smach.StateMachine.add('Navigation', Navigation(),
                               transitions={'failed': 'OHShit',
                                            'success': 'Idle'})

        smach.StateMachine.add('OHShit', OHShit(),
                               transitions={'success': 'Navigation'})

        smach.StateMachine.add('Idle', Idle(),
                               transitions={'success': 'Navigation',
                                            'done': 'Complete'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer(
        'my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()


# class Discover(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeeded','fail'])
#         self.counter = 0
#         self.pose = PoseStamped()
#         #rospy.init_node('check_odometry', anonymous=True)
#         self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)
#         self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
#         self.done = False
#         self.rate = rospy.Rate(0.25)
#         self.cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)

#     def callback(self, msg):

#         #self.pose.pose = msg.pose.pose #strangely, passing 0 starting pose as arg works...
#         self.pose.header.frame_id = "map"
#         now = rospy.get_rostime()
#         self.pose.header.stamp.secs = now.secs
#         self.pose.header.stamp.nsecs = now.nsecs

#     def execute(self, userdata):
#         rospy.loginfo('Executing state Discover')
#         uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#         roslaunch.configure_logging(uuid)
#         launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ghost/catkin_ws/src/asl_turtlebot/launch/frontier_expl.launch"])
#         launch.start()
#         rospy.loginfo("Launching unknown exploration")
#         rospy.sleep(3)
#         while(self.done == False):
#             rospy.wait_for_service('/explore_server/explore_costmap/explore_boundary/get_next_frontier')
#             try:
#                 gnf = rospy.ServiceProxy('/explore_server/explore_costmap/explore_boundary/get_next_frontier', GetNextFrontier)
#                 resp1 = gnf(self.pose)
#                 rospy.loginfo("publishing new frontier")
#                 #rospy.loginfo(resp1.next_frontier)
#                 self.rate.sleep()
#                 self.pub.publish(resp1.next_frontier)
#             except rospy.ServiceException, e:
#                 rospy.loginfo("get_next_frontier unavailable, fully discovered?")
#                 self.done = True

#         if (self.done == True):
#             goalId = GoalID()
#             self.cancel_pub.publish(goalId)
#             rospy.loginfo("Service dropped, map assumed discovered 1")
#             launch.shutdown()
#             rospy.sleep(3)
#             return 'succeeded'
#         else:
#             return 'fail'


# # define state Explore
# class Explore(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeeded','fail'])
#         self.counter = 0
#         self.pose = PoseStamped()
#         #rospy.init_node('check_odometry', anonymous=True)
#         self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)
#         self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
#         self.done = False
#         self.rate = rospy.Rate(0.25)
#         self.doneObserving = False
#         self.cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
#         self.order_Sub = rospy.Subscriber('/food_order', Odometry, self.callback)


#     def callback(self, msg):

#         #self.pose.pose = msg.pose.pose #strangely, passing 0 starting pose as arg works...
#         self.pose.header.frame_id = "map"
#         now = rospy.get_rostime()
#         self.pose.header.stamp.secs = now.secs
#         self.pose.header.stamp.nsecs = now.nsecs

#     def execute(self, userdata):
#         rospy.loginfo('Executing state Explore')
#         uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#         roslaunch.configure_logging(uuid)
#         launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ghost/catkin_ws/src/asl_turtlebot/launch/known_expl.launch"])
#         launch.start()
#         rospy.loginfo("Launching known exploration")
#         rospy.sleep(3)

#         while(self.done == False):
#             rospy.wait_for_service('/explore_server/explore_costmap/explore_boundary/get_next_frontier')
#             try:
#                 gnf = rospy.ServiceProxy('/explore_server/explore_costmap/explore_boundary/get_next_frontier', GetNextFrontier)
#                 resp1 = gnf(self.pose)
#                 rospy.loginfo("publishing new frontier")
#                 #rospy.loginfo(resp1.next_frontier)
#                 self.rate.sleep()
#                 self.pub.publish(resp1.next_frontier)
#             except rospy.ServiceException, e:
#                 rospy.loginfo("get_next_frontier unavailable, looped once")
#                 self.done = True


#         if (self.done == True):
#             rospy.loginfo("Service dropped, succeeded")
#             launch.shutdown()
#             rospy.sleep(3)
#         else:
#             return 'fail'

#         while(self.doneObserving == False):
#             rospy.loginfo("drop 2D nav goal to observe remaining vendors")
#             self.doneObserving = True
#             rospy.loginfo("assuming all observed..")
#             goalId = GoalID()
#             self.cancel_pub.publish(goalId)


#         if (self.doneObserving == True):
#             rospy.sleep(3)
#             return 'succeeded'
#         else:
#             return 'fail'

# class GoHome(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeeded','fail'])
#         self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
#         self.sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.callback )
#         self.reachedGoal = False
#         self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
#         self.client.wait_for_server()
#         self.homePose = MoveBaseGoal()

#     def callback(self, msg):
#         if (msg.status.status == 3):
#             self.reachedGoal = True

#     def execute(self, userdata):
#         rospy.loginfo('Executing state GoHome')

#         now = rospy.get_rostime()
#         self.homePose.target_pose.header.frame_id = "map"
#         self.homePose.target_pose.header.stamp = rospy.Time.now()

#         self.homePose.target_pose.pose.position.x = 3.07134795189
#         self.homePose.target_pose.pose.position.y = 1.6518163681
#         self.homePose.target_pose.pose.position.z = 0
#         self.homePose.target_pose.pose.orientation.x = 0
#         self.homePose.target_pose.pose.orientation.y = 0
#         self.homePose.target_pose.pose.orientation.z = -0.670949053335
#         self.homePose.target_pose.pose.orientation.w = 0.741503450989

#         self.client.send_goal(self.homePose)

#         wait = self.client.wait_for_result()
#         if not wait:
#             rospy.logerr("Action server not available!")
#             rospy.signal_shutdown("Action server not available!")

#         while(self.reachedGoal == False):
#             rospy.sleep(1)
#             print("endless")

#         if (self.reachedGoal == True): #if statement placeholder for false case
#             rospy.loginfo("reached the goal..")
#             return 'succeeded'
#         else:
#             return 'false'

# # main
# def main():
#     rospy.init_node('smach_example_state_machine')

#     # Create a SMACH state machine
#     sm = smach.StateMachine(outcomes=['incomplete', 'complete'])

#     # Open the container
#     with sm:
#         # Add states to the container
#         smach.StateMachine.add('Discover', Discover(),
#                                transitions={'succeeded':'Explore',
#                                             'fail':'incomplete'})
#         smach.StateMachine.add('Explore', Explore(),
#                                transitions={'succeeded':'GoHome',
#                                             'fail':'incomplete'})
#         smach.StateMachine.add('GoHome', GoHome(),
#                                transitions={'succeeded':'complete',
#                                             'fail':'incomplete'})

#     # Create and start the introspection server
#     sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
#     sis.start()

#     # Execute SMACH plan
#     outcome = sm.execute()

#     # Wait for ctrl-c to stop the application
#     rospy.spin()
#     sis.stop()

# if __name__ == '__main__':
#     main()
