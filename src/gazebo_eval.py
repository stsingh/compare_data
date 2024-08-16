#!/usr/bin/python3
import rospy
import os
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class DataCompare:
    def __init__(self):
        # Which run are you using for evaluation?
        run = 1
        self.curr_dir = os.path.dirname(os.path.realpath(__file__))

        # Vars for publishing data to cmd_vel
        self.pub_cmds = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_row = 0
        cmd_cols = ["lv_x", "lv_y", "lv_z", "av_x", "av_y", "av_z", "time"]
        self.cmds = pd.read_csv(f'{os.path.dirname(os.path.realpath(__file__))}/cmd_vel/000{run}_cmd_vel.txt', header=None, names=cmd_cols)
        self.stop_cmd = False

        # Vars for reading wheel odom data from csv and finding error
        self.state_data = None
        self.sub_states = rospy.Subscriber('/joint_states', JointState, self.read_state_data)
        self.pub_wheel_error = rospy.Publisher('wheel_error_pub', Float64, queue_size=10)
        self.odom_row = 0
        odom_cols = ["cmd_mode", "velocity left wheel", "measured travel left", "velocity right wheel", "measured travel right", "time"]
        self.odom = pd.read_csv(f'{self.curr_dir}/odometry/000{run}_odom_data.txt', header=None, names=odom_cols)
        self.start_mtl = self.odom["measured travel left"].iloc[0]
        self.start_mtr = self.odom["measured travel right"].iloc[0]
        self.start_vl = self.odom["velocity left wheel"].iloc[0]
        self.start_vr = self.odom["velocity right wheel"].iloc[0]
        self.ang_vel_l = []
        self.ang_vel_r = []
        self.stop_odom = False

        # Vars for reading localization error from csv and finding error
        self.jackal_odom_data = None
        self.sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, self.read_jackal_odom_data)
        self.pub_local_error = rospy.Publisher('local_error_pub', Float64, queue_size=10)
        self.local_row = 0
        local_cols = ["time", "x", "y", "heading"]
        self.local_truth = pd.read_csv(f'{os.path.dirname(os.path.realpath(__file__))}/localization_ground_truth/000{run}_Tr_tile_GT.txt', header=None, names=local_cols)
        self.start_x = self.local_truth["x"].iloc[0]
        self.start_y = self.local_truth["y"].iloc[0]
        self.start_heading = self.local_truth["heading"].iloc[0]
        self.trajectory_x = []
        self.trajectory_y = []
        self.stop_local = False

    def cmd_pub(self, event=None):
        if self.cmd_row < len(self.cmds):
            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.x, cmd.angular.y, cmd.angular.z, _ = self.cmds.iloc[self.cmd_row,:]
            self.pub_cmds.publish(cmd)
            self.cmd_row += 1
        else:
            self.stop_cmd = True

    def read_state_data(self, msg):
        self.state_data = msg

    def wheel_err_pub(self, event=None):
        if self.odom_row < len(self.odom):
            pos = np.array(self.state_data.position)
            vel = np.array(self.state_data.velocity)
            _, odom_vl, odom_mtl, odom_vr, odom_mtr, _ = self.odom.iloc[self.odom_row,:]
            self.odom_row += 1

            #lp = ((pos[0] + pos[2]) / 2)
            #rp = ((pos[1] + pos[3]) / 2)
            lv = ((vel[0] + vel[2]) / 2)
            rv = ((vel[1] + vel[3]) / 2)

            self.ang_vel_r.append(rv)
            self.ang_vel_l.append(lv)

            err = math.sqrt((
                             #((lp + self.start_mtl) - odom_mtl)**2 + 
                             #((rp + self.start_mtr) - odom_mtr)**2 + 
                             ((lv + self.start_vl) - odom_vl)**2 + 
                             ((rv + self.start_vr) - odom_vr)**2)/2 #4
                            )
            self.pub_wheel_error.publish(Float64(err))
        else:
            self.stop_odom = True

    def read_jackal_odom_data(self, msg):
        self.jackal_odom_data = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    
    def local_err_pub(self, event=None):
        if self.local_row < len(self.local_truth):
            curr_x = self.jackal_odom_data[0]
            curr_y = self.jackal_odom_data[1]
            _, x, y, heading = self.local_truth.iloc[self.local_row,:]
            self.local_row += 1

            self.trajectory_x.append(curr_x)
            self.trajectory_y.append(curr_y)

            err = math.sqrt((
                             ((curr_x + self.start_x) - x)**2 + 
                             ((curr_y + self.start_y) - y)**2)/2
                            )
            self.pub_local_error.publish(err)
        else:
            self.stop_local = True

    def gen_graphs(self, event=None):
        # TODO: 
        # Log trajectories, joint states, and times
        ang_vel_time = np.arange(len(self.ang_vel_l)) / 20.0
        traj_time = np.arange(len(self.trajectory_x)) / 30.0
        fig, (ax1, ax2) = plt.subplots(1, 2)
        ax1.plot(ang_vel_time, self.ang_vel_l, label="Ang Vel Left Wheel")
        ax1.plot(ang_vel_time, self.ang_vel_r, label="Ang Vel Right Wheel")
        ax1.set_title("Angular Velocity Plots")
        ax2.plot(traj_time, self.trajectory_x, label="X Trajectory")
        ax2.plot(traj_time, self.trajectory_y, label="Y Trajectory")
        ax2.set_title("Trajectory Plots")

        filename = f"{self.curr_dir}/output/plots_run"
        i = 0
        while os.path.exists(f"{filename}{i}.jpg"):
            i += 1

        fig.savefig(f"{filename}{i}.jpg")
        

if __name__ == '__main__':
    rospy.init_node("cmd_pub", anonymous=True)
    dc = DataCompare()

    # Timer for publishing data to cmd_vel
    cmd_timer = rospy.Timer(rospy.Duration(1.0/45.0), dc.cmd_pub)
    if dc.stop_cmd:
        cmd_timer.shutdown()

    # Timer for getting wheel error
    wheel_err_timer = rospy.Timer(rospy.Duration(1.0/20.0), dc.wheel_err_pub)
    if dc.stop_odom:
        wheel_err_timer.shutdown()

    # Timer for getting localization error
    local_err_timer = rospy.Timer(rospy.Duration(1.0/30.0), dc.local_err_pub)
    if dc.stop_local:
        local_err_timer.shutdown()
    
    # continue running the program
    rospy.spin()

    # generate the graphs on shutdown
    rospy.on_shutdown(dc.gen_graphs)
