#!/usr/bin/python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np
import pandas as pd
import os
import math

class DataCompare:
    def __init__(self):
        # Vars for publishing data to cmd_vel
        self.pub_cmds = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_row = 0
        cmd_cols = ["lv_x", "lv_y", "lv_z", "av_x", "av_y", "av_z", "time"]
        self.cmds = pd.read_csv(f'{os.path.dirname(os.path.realpath(__file__))}/cmd_vel/0001_cmd_vel.txt', header=None, names=cmd_cols)

        # Vars for reading odom data from csv and finding error
        self.state_data = None
        self.sub_states = rospy.Subscriber('/joint_states', JointState, self.read_state_data)
        self.pub_error = rospy.Publisher('error_pub', Float64, queue_size=10)
        self.odom_row = 0
        odom_cols = ["cmd_mode", "velocity left wheel", "measured travel left", "velocity right wheel", "measured travel right", "time"]
        self.odom = pd.read_csv(f'{os.path.dirname(os.path.realpath(__file__))}/odometry/0001_odom_data.txt', header=None, names=odom_cols)

    def cmd_pub(self, event=None):
        rate = rospy.Rate(45) # Same as paper
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.x, cmd.angular.y, cmd.angular.z, _ = self.cmds.iloc[self.cmd_row,:]
        self.pub_cmds.publish(cmd)
        rate.sleep()
        self.cmd_row += 1

    def read_state_data(self, msg):
        self.state_data = msg

    def err_pub(self, event=None):
        pos = np.array(self.state_data.position)
        vel = np.array(self.state_data.velocity)
        _, odom_vl, odom_mtl, odom_vr, odom_mtr, _ = self.odom.iloc[self.odom_row,:]
        self.odom_row += 1

        lp = ((pos[0] + pos[2]) / 2) / 0.098
        rp = ((pos[1] + pos[3]) / 2) / 0.098
        lv = ((vel[0] + vel[2]) / 2) / 0.098
        rv = ((vel[1] + vel[3]) / 2) / 0.098

        err = math.sqrt(((lp - odom_mtl)**2 + (rp - odom_mtr)**2 + (lv - odom_vl)**2 + (rv - odom_vr)**2)/4)
        self.pub_error.publish(Float64(err))


if __name__ == '__main__':
    rospy.init_node("cmd_pub", anonymous=True)
    dc = DataCompare()

    # Timer for publishing data to cmd_vel
    cmd_timer = rospy.Timer(rospy.Duration(1.0/45.0), dc.cmd_pub)
    if dc.cmd_row >= len(dc.cmds):
        cmd_timer.shutdown()

    # Timer for getting error
    err_timer = rospy.Timer(rospy.Duration(1.0/20.0), dc.err_pub)
    if dc.odom_row >= len(dc.odom):
        err_timer.shutdown()
    rospy.spin()
