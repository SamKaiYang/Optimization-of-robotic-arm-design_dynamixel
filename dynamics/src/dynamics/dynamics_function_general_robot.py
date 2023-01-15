#!/usr/bin/env python3
# coding: utf-8
import importlib
import sys

import rospy

importlib.reload(sys)
import argparse
import csv
import math
import time
from collections import namedtuple
from math import pi
from os import path

import geometry_msgs.msg
import matplotlib.pyplot as plt
import numpy as np
import openpyxl
import plotly.graph_objs as go
import plotly.offline as py
import roboticstoolbox as rtb
import sympy as sp
# import dyna_space
from interface_control.msg import (arm_structure, cal_cmd, cal_process, cal_result, dyna_data,
                                dyna_space_data, optimal_design,
                                optimal_random, specified_parameter_design)
from matplotlib import cm
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from mpl_toolkits.mplot3d import Axes3D
from openpyxl import Workbook
from openpyxl.comments import Comment
from openpyxl.drawing.image import Image
from openpyxl.styles import Alignment, Font, colors
from openpyxl.utils import get_column_letter
from plotly.offline import download_plotlyjs, iplot, plot
from scipy.interpolate import make_interp_spline  # draw smooth
from spatialmath import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

np.set_printoptions(
    linewidth=100,
    formatter={"float": lambda x: f"{x:8.4g}" if abs(x) > 1e-10 else f"{0:8.4g}"},
)

import pandas as pd

from arm_workspace import arm_workspace_plane
# from robot_urdf import RandomRobot
from motor_module import motor_data
from random_robot import RandomRobot
from UR_robot import UR5, UR10, UR10e, UR16e
from TM_robot import TM5_700, TM5_900, TM12, TM14
from single_arm import single_arm


from dynamics_function_6dof_random import Dynamics_random
from dynamics_function_teco import Dynamics_teco
from dynamics_function_single_arm import Dynamics_single_arm
from dynamics_function_tm import Dynamics_tm
from dynamics_function_ur import Dynamics_ur
from dynamics_function_6dof_dynamixel import Dynamics_dynamixel

class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration

    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args:  # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False

class General_arm:
    def __init__(self):
        self.test = 0
        self.op_dof = 0
        self.DoF_select = 0
        self.Structure_select = " "
        # callback:Enter the parameters of the algorithm to be optimized on the interface
        self.sub_optimal_design = rospy.Subscriber(
            "/optimal_design", optimal_design, self.optimal_design_callback
        )
        # callback:Randomly generated shaft length due to optimization algorithm
        self.sub_optimal_random = rospy.Subscriber(
            "/optimal_random", optimal_random, self.optimal_random_callback
        )
        
        self.sub_arm_structure = rospy.Subscriber(
            "/arn_structure", arm_structure, self.arm_structure_callback
        )

    def robot_motor_random_build(self):
        self.robot.__init__()
        print("robot rebuild")
        motor = motor_data()
        # print(motor.TECO_member.head())
        # print(motor.TECO_member.groupby("rated_torque").mean())
        print(
            pd.concat(
                [
                    motor.TECO_member,
                    motor.Kollmorgen_member,
                    motor.UR_member,
                    motor.TM_member,
                ],
                axis=0,
            )
        )

        res = motor.TECO_member.append(other=motor.Kollmorgen_member, ignore_index=True)
        print(res)

        res = motor.TECO_member.append(
            [motor.Kollmorgen_member, motor.UR_member, motor.TM_member],
            ignore_index=True,
        )
        print(res)

        self.robot.plot(self.qn)



    def optimal_design_callback(self, data):
        # print(data.data)
        self.op_dof = data.dof
        self.op_payload = data.payload
        self.op_payload_position = data.payload_position
        self.op_vel = data.vel
        self.op_acc = data.acc
        self.op_radius = data.radius
        

        rospy.loginfo("I heard op_dof is %s", self.op_dof)
        rospy.loginfo("I heard op_payload is %s", self.op_payload)
        rospy.loginfo("I heard op_payload_position is %s", self.op_payload_position)
        rospy.loginfo("I heard op_vel is %s", self.op_vel)
        rospy.loginfo("I heard op_acc is %s", self.op_acc)
        rospy.loginfo("I heard op_radius is %s", self.op_radius)

        # print(self.optimal_design_flag)

    def optimal_random_callback(self, data):
        self.op_axis_2_length = data.axis_2_length
        self.op_axis_3_length = data.axis_3_length

        rospy.loginfo("I heard op_axis_2_length is %s", self.op_axis_2_length)
        rospy.loginfo("I heard op_axis_3_length is %s", self.op_axis_3_length)

    def arm_structure_callback(self, data):
        self.DoF_select = data.DoF
        self.Structure_select = data.structure_name
        
        rospy.loginfo("I heard DoF is %s", self.DoF_select)
        rospy.loginfo("I heard structure_name is %s", self.Structure_select)
        
if __name__ == "__main__":
    rospy.init_node("dynamics_generator")

    general_arm = General_arm()
    while not rospy.is_shutdown():
        # Dya.task_set()
        if general_arm.DoF_select == 0:
            pass
        elif general_arm.DoF_select == 1:
            pass
        elif general_arm.DoF_select == 2:
            pass
        elif general_arm.DoF_select == 3:
            pass
        elif general_arm.DoF_select == 4:
            if general_arm.Structure_select == "SCARA":
                pass
            else:
                pass
        elif general_arm.DoF_select == 5:
            pass
        elif general_arm.DoF_select == 6:
            if general_arm.Structure_select == "TECO":
                robot = Dynamics_teco()
                while general_arm.Structure_select == "TECO":
                    robot.task_set()
            elif general_arm.Structure_select == "UR":
                robot = Dynamics_ur()
                while general_arm.Structure_select == "UR":
                    robot.task_set()
            elif general_arm.Structure_select == "TM":
                robot = Dynamics_tm()
                while general_arm.Structure_select == "TM":
                    robot.task_set()
            elif general_arm.Structure_select == "random":
                robot = Dynamics_random()
                while general_arm.Structure_select == "random":
                    robot.task_set()
            else:
                pass
            
            # print("66666 break") 
            
        elif general_arm.DoF_select == 7:
            if  general_arm.Structure_select == "Single Arm":
                robot = Dynamics_single_arm()
                while general_arm.Structure_select == "Single Arm":
                    robot.task_set()
            elif general_arm.Structure_select == "Panda":
                pass
            else:
                pass
            # print("77777 break") 

