#! /usr/bin/env python

import rospy
from math import radians, degrees, cos, sin, tan, pi, atan2, sqrt
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseWithCovariance, Twist, Point, Pose, PoseArray, PoseStamped

class WaypointNav():
    def __init__(self):
        
        self.current_pose = [None, None, None]
        # Subscribe to Odometry topic to read the current pose of the robot
        rospy.Subscriber("base_pose_ground_truth", Odometry, self.PoseFromOdom)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))

        goal = MoveBaseGoal()
        self.move_base.send_goal(goal)
        Complete = self.move_base.wait_for_result(rospy.Duration(1.0))
        self.move_base.cancel_goal()
    
    def cancelGoal(self):
        print("interrupt, cancelling waypoint")
        self.move_base.cancel_goal()

    # Generate pose (position + orientation)
    def ComputePose(self, pose):
        x = pose[0]
        y = pose[1]
        z = 0
        yaw = pose[2]*pi/180
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        return Pose( Point(x, y, z) , Quaternion(*quat.tolist()) )

    # Read current position and find relative nearest waypoint
    def ComputeDistance(self, waypoint):
        while (self.current_pose[0] == None and not rospy.is_shutdown()):
          rate = rospy.Rate(1)
          rate.sleep()

        x = self.current_pose[0] - waypoint[0]
        y = self.current_pose[1] - waypoint[1]
        return sqrt(x*x + y*y)

    # Navigate to the nearest waypoint
    def Navigate(self, x, y, angle):
        self.move_base.cancel_goal()

        pose = [x, y, angle]

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.ComputePose(pose)
        rospy.loginfo("Moving to the next waypoint!")
        # Send the goal and strat moving the robot
        self.move_base.send_goal(goal)
        # Wait for 2 minutes to reach destination
        Complete = self.move_base.wait_for_result(rospy.Duration(120))

        if not Complete:
            self.move_base.cancel_goal()
            rospy.loginfo("Robot could not reach destination in 2 minutes, Ignoring waypoint!")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Robot reached the destination!")

    # Read the current pose from odometry message
    def PoseFromOdom(self, msg):
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y
        self.current_pose[2] = 0
