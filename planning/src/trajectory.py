#!/usr/bin/env python2
# built-in
import numpy as np

# ros
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class Trajectory:
    def __init__(self):
        rospy.init_node("trajectory_planning")
        self.idx = 0
        self.horizon = 20
        self.num_points = 1000
        self.traj_pub = rospy.Publisher('/waypoints', Path, queue_size=10)
        self.set_global_path("circle")  # type: circle, infinity

    def set_global_path(self, type):
        if type == "circle":
            radius = 8
            x, y = 8, 0   # center of circle
            theta = np.linspace(-np.pi, np.pi, self.num_points)
            self.waypoints = np.array([(radius * np.cos(angle) + x, radius * np.sin(angle) + y) for angle in theta])
        elif type == "infinity":
            a = 20  # scaling facator
            theta = np.linspace(0, 2 * np.pi, self.num_points)
            self.waypoints = np.array([(a * np.sin(angle), a * np.sin(angle) * np.cos(angle)) for angle in theta])

    def publish_waypoints(self):
        path = Path()
        path.header.frame_id = "base_link"
        path.header.stamp = rospy.Time.now()

        end_idx = self.idx + self.horizon
        if end_idx >= len(self.waypoints):
            self.idx = 0
            end_idx = self.idx + self.horizon

        for point in self.waypoints[self.idx:end_idx]:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0  # No rotation
            path.poses.append(pose_stamped)

        self.traj_pub.publish(path)

        self.idx += 1

    # def visualize_waypoints(self):
    #     path = Path()
    #     path.header.frame_id = "base_link"
    #     path.header.stamp = rospy.Time.now()
    #
    #     for point in self.waypoints:
    #         pose_stamped = PoseStamped()
    #         pose_stamped.header.frame_id = "base_link"
    #         pose_stamped.header.stamp = rospy.Time.now()
    #         pose_stamped.pose.position.x = point[0]
    #         pose_stamped.pose.position.y = point[1]
    #         pose_stamped.pose.position.z = 0.0
    #         pose_stamped.pose.orientation.w = 1.0  # No rotation
    #         path.poses.append(pose_stamped)
    #
    #     self.vis_pub.publish(path)


if __name__ == "__main__":
    planner = Trajectory()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        planner.publish_waypoints()
        rate.sleep()