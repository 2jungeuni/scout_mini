#!/usr/bin/env python2
# built-in
import numpy as np

# ros
import rospy
from visualization_msgs.msg import Marker


class Trajectory:
    def __init__(self, hz=10):
        self.marker_id = 0
        self.num_points = 100
        self.marker_pub = rospy.Publisher('/transformed_points_marker', Marker, queue_size=self.num_points)
        self.set_global_path()

    def set_global_path(self):
        radius = 8
        x, y = 8, 0   # center of circle
        theta = np.linspace(-np.pi, np.pi, self.num_points)
        self.path = np.array([(radius * np.cos(angle) + x, radius * np.sin(angle) + y) for angle in theta])
        self.publish_waypoints(self.path)

    def publish_waypoints(self, points):
        for point in points:
            print(point)
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.id = self.marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            self.marker_pub.publish(marker)

            self.marker_id += 1


if __name__ == "__main__":
    hz = 50
    rospy.init_node("trajectory_planning")
    node = Trajectory(hz=hz)
    rate = rospy.Rate(hz)                   # 50 hz
    while not rospy.is_shutdown():
        rate.sleep()