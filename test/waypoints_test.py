#!/usr/bin/env python

import rospy
import unittest
import rostest
import actionlib
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal
import math
from tf import transformations

class TestWaypointAction(unittest.TestCase):
    def __init__(self, *args):
        super(TestWaypointAction, self).__init__(*args)
        rospy.init_node('test_waypoint_action', anonymous=True)
        self.odom_data = None
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(1)
        #self.goal_position = Point(x=0.25, y=0.5, z=0.0)
        #self.goal_position = Point(x=0.0, y=0.0, z=0.0)
        self.goal_position = Point(x=100.0, y=0.0, z=0.0)
        self.goal_yaw = math.atan2(self.goal_position.y, self.goal_position.x)

        # Create action client
        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        self.client.wait_for_server()
        self.result = None

    def odom_callback(self, msg):
        self.odom_data = msg

    def test_end_position(self):
        # Send goal to action server
        goal = WaypointActionGoal()
        goal.position = self.goal_position
        self.client.send_goal(goal)

        # Wait for the action server to complete the goal with a timeout of 30 seconds
        self.result = self.client.wait_for_result(rospy.Duration(30.0))

        # Check if the result was received within the timeout
        self.assertTrue(self.result, "Action server did not complete the goal in time")

        self.assertIsNotNone(self.odom_data, "Odometry data not received")
        current_position = self.odom_data.pose.pose.position
        error_margin = 0.05  # Allowable error margin for position

        self.assertAlmostEqual(current_position.x, self.goal_position.x, delta=error_margin, 
                               msg="Final X position is incorrect")
        self.assertAlmostEqual(current_position.y, self.goal_position.y, delta=error_margin, 
                               msg="Final Y position is incorrect")

    def test_end_yaw(self):
        self.assertIsNotNone(self.odom_data, "Odometry data not received")
        orientation = self.odom_data.pose.pose.orientation
        _, _, current_yaw = self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        error_margin = math.pi / 90  # Allowable error margin for yaw

        self.assertAlmostEqual(current_yaw, self.goal_yaw, delta=10*error_margin, 
                               msg="Final Yaw is incorrect")

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        return roll, pitch, yaw

if __name__ == '__main__':
    rostest.rosrun('tortoisebot_waypoints', 'test_waypoint_action', TestWaypointAction)
