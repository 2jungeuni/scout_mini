# built-in
import os
import sys
import math
import numpy as np

# optimizer
import gurobipy as gp
from gurobipy import *

# ros
import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from controller.msg import RefPoses
from visualization_msgs.msg import Marker

# visualization test
# import matplotlib.pyplot as plt


# # status dictionary
# status_dict = {1: "loaded",
#                2: "optimal",
#                3: "infeasible",
#                4: "infeasible and unbounded",
#                5: "unbounded",
#                6: "cut off",
#                7: "iteration limit",
#                8: "node limit",
#                9: "time limit",
#                10: "solution limit",
#                11: "interrupted",
#                12: "numeric",
#                13: "suboptimal",
#                14: "in progress",
#                15: "user objective limit",
#                16: "work limit",
#                17: "memory limit"}


class MPCController:
    def __init__(self, hz=50, horizon=10):
        self.num_points = 1000
        # self.marker_pub = rospy.Publisher('/waypoints_marker', Marker, queue_size=1)
        # self.set_global_path()
        # rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # rospy.Subscriber("./ref_pos", RefPoses, self.waypoints_callback)

        self.marker_id = 0

    def set_global_path(self):
        # radius = 800
        # x, y = 800, 0   # center of circle
        # theta = np.linspace(-np.pi, np.pi, self.num_points)
        # self.path = np.array([(radius * np.cos(angle) + x, radius * np.sin(angle) + y) for angle in theta])[50:]
        self.publish_waypoints([(0, 0)])

    def odom_callback(self, data):
        self.odom_pose = data.pose.pose
        self.odom_twist = data.twist.twist

    def waypoints_callback(self, data):
        self.current_x, self.current_y = self.odom_pose.position.x, self.odom_pose.position.y
        self.current_theta = self.get_yaw_from_quaternion(self.odom_pose.orientation)


    def get_yaw_from_quaternion(self, q):
        import tf.transformations
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]

    def publish_waypoints(self, points):
        for point in points:
            marker = Marker()
            marker.header.frame_id = "waypoint"
            marker.header.stamp = rospy.Time.now()
            marker.id = self.marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            self.marker_pub.publish(marker)

            self.marker_id += 1



if __name__ == "__main__":
    hz = 50
    rospy.init_node("MPC")
    node = MPCController(hz=hz, horizon=10)
    rate = rospy.Rate(hz)       # 50 hz
    while not rospy.is_shutdown():
        rate.sleep()