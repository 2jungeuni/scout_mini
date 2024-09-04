# built-in
import os
import sys
import math
import numpy as np
from collections import deque, OrderedDict

# optimizer
import gurobipy as gp

# ros
import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker


# status dictionary
status_dict = {1: "loaded",
               2: "optimal",
               3: "infeasible",
               4: "infeasible and unbounded",
               5: "unbounded",
               6: "cut off",
               7: "iteration limit",
               8: "node limit",
               9: "time limit",
               10: "solution limit",
               11: "interrupted",
               12: "numeric",
               13: "suboptimal",
               14: "in progress",
               15: "user objective limit",
               16: "work limit",
               17: "memory limit"}


class MPCController:
    def __init__(self, hz=50, horizon=10, q_len=20):
        self.horizon = horizon
        self.q_len = q_len
        self.dt = 1 / hz

        rospy.init_node("mpc_controller")
        self.waypoints_buffer = deque(maxlen=q_len)
        self.waypoints_dict = OrderedDict()

        # subscriber
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/waypoints", Path, self.waypoints_callback)

        # publisher
        self.cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def odom_callback(self, data):
        self.odom_pose = data.pose.pose
        self.odom_twist = data.twist.twist
        self.heading = self.get_yaw_from_quaternion(self.odom_pose.orientation)
        self.current_x, self.current_y = self.odom_pose.position.x, self.odom_pose.position.y
        self.run_mpc()

    def waypoints_callback(self, data):
        first_waypoint = (data.poses[0].pose.position.x, data.poses[0].pose.position.y)
        self.waypoints_buffer.append(first_waypoint)
        self.waypoints_dict[first_waypoint] = [(pose_stamp.pose.position.x, pose_stamp.pose.position.y)
                                               for pose_stamp in data.poses]

        if len(self.waypoints_dict) > self.q_len:
            self.waypoints_dict.popitem(last=False)

    def run_mpc(self):
        closest = self.get_closest()
        waypoints = self.waypoints_dict[closest]

        if self.get_distance(closest[0], closest[1], self.current_x, self.current_y) < 30:
            # define optimization model
            m = gp.Model()
            m.Params.outputFlag = False

            # define variables
            # x, y variables
            x_vars = m.addVars(np.arange(self.horizon), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY, vtype=gp.GRB.CONTINUOUS, name="x")
            y_vars = m.addVars(np.arange(self.horizon), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY, vtype=gp.GRB.CONTINUOUS, name="y")
            # v variables
            vx_vars = m.addVars(np.arange(self.horizon - 1), lb=-2, ub=2, vtype=gp.GRB.CONTINUOUS, name="v_x")
            vy_vars = m.addVars(np.arange(self.horizon - 1), lb=-2, ub=2, vtype=gp.GRB.CONTINUOUS, name="v_y")
            # omega variables
            omgx_vars = m.addVars(np.arange(self.horizon), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY, vtype=gp.GRB.CONTINUOUS,
                                  name="omg_x")
            omgy_vars = m.addVars(np.arange(self.horizon), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY, vtype=gp.GRB.CONTINUOUS,
                                  name="omg_y")

            # define the constraints
            # Constraint 1: set initial points
            cons1_1 = m.addConstr(x_vars[0] == self.current_x)
            cons1_2 = m.addConstr(y_vars[0] == self.current_y)

            # Constraint 2: dynamics
            cons2_1 = m.addConstrs(x_vars[h + 1] == x_vars[h] + self.dt * vx_vars[h] for h in range(self.horizon - 1))
            cons2_2 = m.addConstrs(y_vars[h + 1] == y_vars[h] + self.dt * vy_vars[h] for h in range(self.horizon - 1))
            cons2_3 = m.addConstrs(vx_vars[h + 1] == vx_vars[h] + self.dt * omgx_vars[h] for h in range(self.horizon - 2))
            cons2_4 = m.addConstrs(vy_vars[h + 1] == vy_vars[h] + self.dt * omgy_vars[h] for h in range(self.horizon - 2))

            # set objective function
            m.setObjective(gp.quicksum((waypoints[h][0] - x_vars[h]) ** 2 for h in range(self.horizon))
                           + gp.quicksum((waypoints[h][1] - y_vars[h]) ** 2 for h in range(self.horizon))
                           + gp.quicksum((vx_vars[h + 1] - vx_vars[h]) ** 2 for h in range(self.horizon - 2))
                           + gp.quicksum((vy_vars[h + 1] - vy_vars[h]) ** 2 for h in range(self.horizon - 2))
                           + gp.quicksum((omgx_vars[h + 1] - omgx_vars[h]) ** 2 for h in range(self.horizon - 2))
                           + gp.quicksum((omgy_vars[h + 1] - omgy_vars[h]) ** 2 for h in range(self.horizon - 2)),
                           gp.GRB.MINIMIZE)

            m._xvars = x_vars
            m._yvars = y_vars
            m._vxvars = vx_vars
            m._vyvars = vy_vars
            m._omgxvars = omgx_vars
            m._omgyvars = omgy_vars
            m.optimize()

            print("Solved (%s)" % status_dict[m.status])

            if m.status == 2:
                print("Objective value: ", m.ObjVal)
                x_vals = m.getAttr('x', x_vars)
                y_vals = m.getAttr('x', y_vars)
                vx_vals = m.getAttr('x', vx_vars)
                vy_vals = m.getAttr('x', vy_vars)
                omgx_vals = m.getAttr('x', omgx_vars)
                omgy_vals = m.getAttr('x', omgy_vars)
                theta = 0.0 if np.round(vx_vals[0], 5) == 0 else np.tanh(np.round(vy_vals[0], 5) / np.round(vx_vals[0], 5))

                ctrl_cmd = Twist()
                ctrl_cmd.linear.x = np.round(vx_vals[0], 5)
                ctrl_cmd.angular.z = theta
                self.cmd.publish(ctrl_cmd)
            else:
                sys.exit()
        else:
            ctrl_cmd = Twist()
            ctrl_cmd.linear.x = 1.0
            ctrl_cmd.angular.z = 0.0
            self.cmd.publish(ctrl_cmd)

    def get_distance(self, x0, y0, x1, y1):
        return math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)

    def get_closest(self):
        return min(self.waypoints_buffer, key=lambda point: self.get_distance(point[0], point[1], self.current_x, self.current_y))

    def get_yaw_from_quaternion(self, q):
        import tf.transformations
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]


if __name__ == "__main__":
    hz = 50
    mpc = MPCController(hz=hz, horizon=10)
    rate = rospy.Rate(hz)       # 50 hz
    while not rospy.is_shutdown():
        rate.sleep()