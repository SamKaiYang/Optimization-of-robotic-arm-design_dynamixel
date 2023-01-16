#!/usr/bin/env python3
# coding: utf-8
import gym
from gym import spaces
import numpy as np
from math import pi

from sympy import false
# from torch import R
from interface_control.msg import optimal_design
from dynamics.arm_workspace import arm_workspace_plane
# from robot_urdf import RandomRobot
from dynamics.motor_module import motor_data
from dynamics.modular_robot_6dof import modular_robot_6dof
# from dynamics.stl_conv_6dof_urdf import stl_conv_urdf
from dynamics.stl_conv_6dof_urdf_dynamixel import stl_conv_urdf
import rospy
# one-hot encoder
from sklearn import preprocessing
import pandas as pd
import yaml
# add 可達性可操作性評估
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import spatialmath as sm
from urdf_parser_py.urdf import URDF
import os
from openpyxl import load_workbook
from openpyxl import Workbook

# TODO: 初版 只考慮 6 dof 機器人的關節長度變化, 觀察各軸馬達極限之輸出最大torque值
class RobotOptEnv(gym.Env):
    """
    ### Description
    This environment corresponds to the version of the cart-pole problem described by Barto, Sutton, and Anderson in
    ["Neuronlike Adaptive Elements That Can Solve Difficult Learning Control Problem"](https://ieeexplore.ieee.org/document/6313077).
    A pole is attached by an un-actuated joint to a cart, which moves along a frictionless track.
    The pendulum is placed upright on the cart and the goal is to balance the pole by applying forces
     in the left and right direction on the cart.
    ### Action Space
    The action is a `ndarray` with shape `(1,)` which can take values `{0, 1}` indicating the direction
     of the fixed force the cart is pushed with.
    | Num | Action                 |
    |-----|------------------------|
    | 0   | length add 1 cm  |
    | 1   | length del 1 cm |
    **Note**: The velocity that is reduced or increased by the applied force is not fixed and it depends on the angle
     the pole is pointing. The center of gravity of the pole varies the amount of energy needed to move the cart underneath it
    ### Observation Space
    The observation is a `ndarray` with shape `(4,)` with the values corresponding to the following positions and velocities:
    | Num | Observation           | Min                 | Max               |
    |-----|-----------------------|---------------------|-------------------|
    | 0   | Cart Position         | -4.8                | 4.8               |
    | 1   | Cart Velocity         | -Inf                | Inf               |
    | 2   | Pole Angle            | ~ -0.418 rad (-24°) | ~ 0.418 rad (24°) |
    | 3   | Pole Angular Velocity | -Inf                | Inf               |
    **Note:** While the ranges above denote the possible values for observation space of each element,
        it is not reflective of the allowed values of the state space in an unterminated episode. Particularly:
    -  The cart x-position (index 0) can be take values between `(-4.8, 4.8)`, but the episode terminates
       if the cart leaves the `(-2.4, 2.4)` range.
    -  The pole angle can be observed between  `(-.418, .418)` radians (or **±24°**), but the episode terminates
       if the pole angle is not in the range `(-.2095, .2095)` (or **±12°**)
    ### Rewards
    Since the goal is to keep the pole upright for as long as possible, a reward of `+1` for every step taken,
    including the termination step, is allotted. The threshold for rewards is 475 for v1.
    ### Starting State
    All observations are assigned a uniformly random value in `(-0.05, 0.05)`
    ### Episode End
    The episode ends if any one of the following occurs:
    1. Termination: Pole Angle is greater than ±12°
    2. Termination: Cart Position is greater than ±2.4 (center of the cart reaches the edge of the display)
    3. Truncation: Episode length is greater than 500 (200 for v0)
    """

    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 2
    }
    def __init__(self):
        self.robot = modular_robot_6dof()
        self.robot_urdf = stl_conv_urdf("single_arm_v12","test")
        
        # callback:Enter the parameters of the algorithm to be optimized on the interface
        self.sub_optimal_design = rospy.Subscriber(
            "/optimal_design", optimal_design, self.optimal_design_callback
        )
        # 使用者設定參數
        self.payload = 5.0
        self.payload_position = np.array([0, 0, 0.04])
        self.vel = np.array([2.356194, 2.356194, 2.356194, 2.356194, 2.356194, 2.356194])
        self.acc = np.array([2.356194, 2.356194, 2.356194, 2.356194, 2.356194, 2.356194])
        self.total_weight = 20 # Kg
        self.total_cost = 1800 # 元
        # 預設二,三軸軸長
        self.std_L2 = 35.0 # 預設標準值 第二軸 35 cm
        self.std_L3 = 35.0 # 預設標準值 第三軸 35 cm
        # 觀察參數 motor
        self.motor = motor_data()
        self.res = self.motor.dynamixel_member
        self.high_torque = 120.0 # 預設標準值 馬達極限 120.0 N max
        self.low_torque = 60.0 # 預設標準值 馬達極限 60.0 N rated
        self.motor_cost_init = np.array([0,400,400,0,0,0], dtype=np.float32) # 預設最大馬達費用
        self.motor_weight_init = np.array([0,0.855,0.855,0,0,0], dtype=np.float32) # 預設最大馬達重量
        self.motor_cost = np.array([0,400,400,0,0,0], dtype=np.float32) # 馬達費用
        self.motor_weight = np.array([0,0.855,0.855,0,0,0], dtype=np.float32) # 馬達重量
        self.motor_rated = np.array([44.7,44.7,44.7,44.7,44.7,44.7], dtype=np.float32)
        # 使用者設定參數 & 觀察參數
        self.reach_distance = 0.6 # 使用者設定可達半徑最小值
        self.high_reach_eva = 1 # 預設觀測標準值
        self.low_reach_eva = 0 # 預設觀測標準值
        self.high_manipulability = 1  # 預設觀測標準值
        self.low_manipulability = 0  # 預設觀測標準值
        self.torque_done = np.array([false, false, false, false, false, false])
        self.torque_over = False
        self.prev_shaping = None
        # TODO: 增加馬達模組選型action
        self.action_space = spaces.Discrete(12) # TODO: fixed 12種action
        
        # TODO: observation space for torque, reach, motor cost, weight, manipulability
        self.observation_space = spaces.Box(np.array([self.low_torque,self.low_torque,self.low_torque,self.low_torque,self.low_torque,self.low_torque, self.low_reach_eva, -float('inf'), -float('inf'), self.low_manipulability ]), 
                                            np.array([self.high_torque,self.high_torque,self.high_torque,self.high_torque,self.high_torque,self.high_torque, self.high_reach_eva, self.total_cost, self.total_weight, self.high_manipulability]), 
                                            dtype=np.float32)
        # TODO: reward 歸一化
        self.state = np.array([0,0,0,0,0,0,0,0,0,0], dtype=np.float32)
        self.pre_state = np.array([0,0,0,0,0,0,0,0,0,0], dtype=np.float32)


        #隨機抽樣點位初始化
        self.T_x = []
        self.T_y = []
        self.T_z = []
        self.T_roll = []
        self.T_pitch = []
        self.T_yaw = []
        self.xlsx_outpath = "./xlsx/"

    def optimal_design_callback(self, data):
        # print(data.data)
        # TODO: 目標構型
        self.op_dof = data.dof
        self.op_payload = data.payload
        self.op_payload_position = data.payload_position
        self.op_vel = data.vel
        self.op_acc = data.acc
        self.op_radius = data.radius
        self.op_weight = data.arm_weight
        self.op_cost = data.cost
        
        print("op_dof:", self.op_dof)
        print("op_payload:", self.op_payload)
        print("op_payload_position:", self.op_payload_position)
        print("op_vel:", self.op_vel)
        print("op_acc:", self.op_acc)
        print("op_radius:", self.op_radius)
        print("op_weight:", self.op_weight)
        print("op_cost:", self.op_cost)
        
        self.payload = self.op_payload
        self.payload_position = np.array(self.op_payload_position)
        self.vel = np.array(self.op_vel[0:6])
        self.acc = np.array(self.op_acc[0:6])
        self.total_weight = self.op_weight # Kg
        self.total_cost = self.op_cost # 元
        self.reach_distance = self.op_radius # 使用者設定可達半徑最小值
        
    # TODO: fixed
    def step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))
        self.pre_state[0:6] = self.state[0:6]
        # TODO: 向量編碼動作
        if action == 0: # 軸2  # 短 # 型號1
            self.std_L2 -= 1.0
            # 配置軸2 motor 型號1
            motor_type = 0 # 型號
            axis = 2 # 軸數
            
        elif action == 1: # 軸2  # 長 # 型號1
            self.std_L2 += 1.0
            motor_type = 0 # 型號
            axis = 2 # 軸數
            
        elif action == 2: # 軸2  # 短 # 型號2
            self.std_L2 -= 1.0
            motor_type = 1 # 型號
            axis = 2 # 軸數
            
        elif action == 3: # 軸2  # 長 # 型號2
            self.std_L2 += 1.0
            motor_type = 1 # 型號
            axis = 2 # 軸數
            
        elif action == 4: # 軸2  # 短 # 型號3
            self.std_L2 -= 1.0
            motor_type = 2 # 型號
            axis = 2 # 軸數
            
        elif action == 5: # 軸2  # 長 # 型號3
            self.std_L2 += 1.0
            motor_type = 2 # 型號
            axis = 2 # 軸數
            
        elif action == 6: # 軸3  # 短 # 型號1
            self.std_L3 -= 1.0
            motor_type = 0 # 型號
            axis = 3 # 軸數
            
        elif action == 7: # 軸3  # 長 # 型號1
            self.std_L3 += 1.0
            motor_type = 0 # 型號
            axis = 3 # 軸數
            
        elif action == 8: # 軸3  # 短 # 型號2
            self.std_L3 -= 1.0
            motor_type = 1 # 型號
            axis = 3 # 軸數
            
        elif action == 9: # 軸3  # 長 # 型號2
            self.std_L3 += 1.0
            motor_type = 1 # 型號
            axis = 3 # 軸數
            
        elif action == 10: # 軸3  # 短 # 型號3
            self.std_L3 -= 1.0
            motor_type = 2 # 型號
            axis = 3 # 軸數
            
        elif action == 11: # 軸3  # 長 # 型號3
            self.std_L3 += 1.0
            motor_type = 2 # 型號
            axis = 3 # 軸數
        
        # 輸入action後 二,三軸軸長
        self.robot_urdf.specified_generate_write_urdf(self.std_L2, self.std_L3)
        self.robot.__init__() # 重製機器人
        torque = self.dynamics_torque_limit()
        self.state[0:6] = torque
        # # 可達性
        # self.state[6] = self.reach_evaluate()
        # 計算成本與重量    
        self.motor_cost[axis-1] = self.res.cost[motor_type]
        self.motor_weight[axis-1] = self.res.weight[motor_type]
        # print(self.motor_cost)
        # print(self.motor_weight)
        cost = sum(self.motor_cost) 
        weight = sum(self.motor_weight)
        self.state[7] = cost
        self.state[8] = weight
        
        # self.state[9] = self.manipulability_evaluate()
        # 可達性 # 可操作性
        self.state[6], self.state[9] = self.reach_manipulability_evaluate()
        rospy.loginfo("reach_score: %s", self.state[6])
        # rospy.loginfo("manipulability_score: %s", self.state[9])
        self.motor_rated[axis-1] = self.res.rated_torque[motor_type]
        # rospy.loginfo("configuration cost & weight: %s, %s", cost, weight)
        self.counts += 1
        rospy.loginfo("self.state: %s", self.state)
        reward = 0
        # TODO: fixed
        shaping = (
            # -10 * np.sqrt(self.state[0] * self.state[0] + self.state[1] * self.state[1] + self.state[2] * self.state[2] + self.state[3] * self.state[3] + self.state[4] * self.state[4] + self.state[5] * self.state[5])
            + 10 * self.state[6]
            - 0.1 * self.state[7]
            - 10 * self.state[8]
            + 1000 * self.state[9]
        ) 

        if self.prev_shaping is not None:
            reward = shaping - self.prev_shaping
        self.prev_shaping = shaping

        # 判斷超出最大扭矩
        for i in range(6):
            # TODO:consider cost & weight 
            # if np.abs(self.state[i]) > self.motor_rated[i]:
            if np.abs(self.state[i]) > 44.7: # TODO: fixed 
                self.torque_over = True # 超出最大扭矩
                break # TODO

        terminated = False
        # if down 完成任务 
        if self.state[6] <= 0.6: # TODO: fixed
            terminated = True
            reward += -50
        if self.torque_over == True: # TODO: fixed 0112 00:22 改為超過最大的馬達型號torque
            terminated = True
            reward += -100
        if self.torque_over == False and self.state[6] == 1 and self.state[8] < self.op_weight and self.state[7] < self.op_cost:  # TODO: fixed 增加 cost & weight & ... 
            terminated = True
            reward += +50
        if self.counts == 30:
            terminated = True
            self.counts = 0
            reward += +30
        self.torque_over = False #reset
        rospy.loginfo("counts: %s", self.counts)
        rospy.loginfo("step_reward: %s", reward)
        rospy.loginfo("================================")
        # print("================================")
        return self.state, reward, terminated, {}


    # Define the base reward function
    def base_reward_function(self, joint_torques, joint_costs, arm_weight, manipulability, reachability):
        # Calculate the torque reward
        torque_reward = sum(joint_torques) / len(joint_torques)
        # Calculate the cost reward
        cost_reward = sum(joint_costs) / len(joint_costs)
        # Calculate the weight reward
        weight_reward = arm_weight
        # Calculate the manipulability reward
        manipulability_reward = manipulability
        # Calculate the reachability reward
        reachability_reward = reachability
        # Sum up all rewards and return the final reward
        base_reward = torque_reward + cost_reward + weight_reward + manipulability_reward + reachability_reward
        return base_reward

    # Define the shaping reward function
    def shaping_reward_function(self, current_position, target_position):
        # Calculate the distance between current position and target position
        distance = np.linalg.norm(current_position - target_position)
        # Define the shaping reward
        shaping_reward = - distance
        return shaping_reward

    # Combine the base reward function and the shaping reward function
    def combined_reward_function(self, joint_torques, joint_costs, arm_weight, manipulability, reachability, current_position, target_position):
        base_reward = self.base_reward_function(joint_torques, joint_costs, arm_weight, manipulability, reachability)
        shaping_reward = self.shaping_reward_function(current_position, target_position)
        final_reward = base_reward + shaping_reward
        return final_reward
    # reset环境状态 
    def reset(self):
        # TODO:改用random state (手臂長度隨機)
        # self.robot_urdf.opt_generate_write_urdf() # 啟用標準的L2,L3長度urdf
        self.std_L2, self.std_L3 = self.robot_urdf.opt_random_generate_write_urdf() # 啟用隨機的L2,L3長度urdf
        # TODO:讀取random後的臂長
        self.robot.__init__() # 重製機器人
        torque = self.dynamics_torque_limit()
        self.state[0:6] = torque
        # consider reach_distance
        # L1,L2,L3 = self.robot.return_configuration()
        # L_sum = L1+L2+L3
        self.random_select_point() # 先隨機抽樣30個點位
        # reach_score = self.reach_evaluate()
        # manipulability_score = self.manipulability_evaluate()
        reach_score, manipulability_score = self.reach_manipulability_evaluate()
        self.state[6] = reach_score
        self.state[7] = sum(self.motor_cost_init)
        self.state[8] = sum(self.motor_weight_init)
        self.state[9] = manipulability_score
        # rospy.loginfo("configuration: %s, %s, %s, %s", L1, L2, L3, L_sum)
        
        
        # 重置
        # self.std_L2 = 35.0 # 預設標準值 
        # self.std_L3 = 35.0 # 預設標準值
        
        self.counts = 0
        return self.state    
    
    # 視覺化呈現，它只會回應出呼叫那一刻的畫面給你，要它持續出現，需要寫個迴圈
    def render(self, mode='human'):
        return None
        
    def close(self):
        return None
        
    def dynamics_torque_limit(self):
        """
        Calculate the maximum torque required by

        each axis when the arm of each axis is the longest and the acceleration is the highest
        """
        torque = np.array([np.zeros(shape=6)])
        # axis_angle = np.array([np.zeros(shape=6)])
        axis_angle = []
        append_torque_limit_list = []
        temp_torque_max = []
        temp_torque_min = []
        Torque_Max = []
        # 角度轉換
        du = pi / 180
        # 度
        # radian = 180 / pi
        # 弧度
        self.robot.payload(self.payload, self.payload_position)  # set payload
        torque = np.array([np.zeros(shape=6)])
        q_list = [0, 90, -90, 180, -180]
        T_cell = (
            len(q_list)
            * len(q_list)
            * len(q_list)
            * len(q_list)
            * len(q_list)
            * len(q_list)
        )

        for i in range(len(q_list)):
            q1 = q_list[i]
            percent = i / T_cell * 100
            for j in range(len(q_list)):
                q2 = q_list[j]
                for k in range(len(q_list)):
                    q3 = q_list[k]
                    for l in range(len(q_list)):
                        q4 = q_list[l]
                        for m in range(len(q_list)):
                            q5 = q_list[m]
                            for n in range(len(q_list)):
                                q6 = q_list[n]
                                axis_angle.append([q1, q2, q3, q4, q5, q6])
                                load = np.array(
                                    [
                                        self.robot.rne(
                                            [
                                                q1 * du,
                                                q2 * du,
                                                q3 * du,
                                                q4 * du,
                                                q5 * du,
                                                q6 * du,
                                            ],
                                            self.vel,
                                            self.acc
                                        )
                                    ]
                                )
                                torque = np.append(torque, load, axis=0)

        for i in range(6):
            axis = i
            toque_max_index = np.argmax(torque[:, axis])
            toque_min_index = np.argmin(torque[:, axis])

            temp_torque_max = torque[toque_max_index].tolist()
            temp_torque_max.extend(axis_angle[toque_max_index])
            temp_torque_min = torque[toque_min_index].tolist()
            temp_torque_min.extend(axis_angle[toque_min_index])
            append_torque_limit_list.append(temp_torque_max)
            append_torque_limit_list.append(temp_torque_min)
            Torque_Max.append(abs(torque[toque_max_index][i]))
            self.torque_dynamics_limit = Torque_Max


        # print("torque_dynamics_limit: ", self.torque_dynamics_limit)
        return self.torque_dynamics_limit

    # TODO: 新增可達性評估
    def reach_manipulability_evaluate(self):
        # import xlsx
        df = load_workbook("./xlsx/task_point.xlsx")
        sheets = df.worksheets
        sheet1 = sheets[0]
        rows = sheet1.rows
        cols = sheet1.columns
        T_tmp = []
        score = []
        manipulability_index = []
        i = 0
        false_done = False
        count = 0
        for row in rows:
            row_val = [col.value for col in row]
            T_tmp.append(SE3(row_val[0], row_val[1], row_val[2]) * SE3.RPY([np.deg2rad(row_val[3]), np.deg2rad(row_val[4]), np.deg2rad(row_val[5])]))
            ik_q = self.robot.ikine_LMS(T=T_tmp[i])
            if ik_q.success == True:
                count += 1
                manipulability_index.append(self.robot.manipulability(q=ik_q.q))
            i = i + 1

        if i == 0:
            return(0,0)
        else:
            final_score = count / i
            if i == 1:
                return(final_score, manipulability_index[0]) # 回傳 manipulability[0]
            else:
                return(final_score, np.mean(manipulability_index)) # 回傳 manipulability 取平均

    # def reach_evaluate(self):
    #     # import xlsx
    #     df = load_workbook("./xlsx/task_point.xlsx")
    #     sheets = df.worksheets
    #     sheet1 = sheets[0]
    #     rows = sheet1.rows
    #     cols = sheet1.columns
    #     T_tmp = []
    #     score = []
    #     i = 0
    #     false_done = False
    #     count = 0
    #     for row in rows:
    #         row_val = [col.value for col in row]
    #         T_tmp.append(SE3(row_val[0], row_val[1], row_val[2]) * SE3.RPY([np.deg2rad(row_val[3]), np.deg2rad(row_val[4]), np.deg2rad(row_val[5])]))
    #         ik_q = self.robot.ikine_LM(T=T_tmp[i])
    #         if ik_q.success == True:
    #             count += 1
    #         i = i + 1
    #     final_score = count / i
    #     # TODO: 改為到點率
    #     return(final_score) # 回傳 

    # # TODO: 新增可操作性評估
    # def manipulability_evaluate(self):
    #     # import xlsx
    #     df = load_workbook("./xlsx/task_point.xlsx")
    #     sheets = df.worksheets
    #     sheet1 = sheets[0]
    #     rows = sheet1.rows
    #     cols = sheet1.columns

    #     T_tmp = []
    #     manipulability_index = []
    #     i = 0
    #     for row in rows:
    #         row_val = [col.value for col in row]
    #         T_tmp.append(SE3(row_val[0], row_val[1], row_val[2]) * SE3.RPY([np.deg2rad(row_val[3]), np.deg2rad(row_val[4]), np.deg2rad(row_val[5])]))
    #         # print(T_tmp[i])
    #         ik_q = self.robot.ikine_LM(T=T_tmp[i], search=False, slimit=100)
    #         if ik_q.success == True:
    #             manipulability_index.append(self.robot.manipulability(q=ik_q.q))
    #         i = i + 1
    #     if i == 0:
    #         return(0)
    #     else:
    #         return(np.mean(manipulability_index)) # 回傳 manipulability 取平均
    
    
    def point_Workspace_cal_Monte_Carlo(self):
        """
        Through the "Work space" page in the interface to calculate of the robot
        """
        i = 0
        # 角度轉換
        du = pi / 180
        # 度
        radian = 180 / pi
        # 弧度

        # fig = plt.figure()
        # self.ax = plt.subplot(111, projection='3d')
        # self.ax_2d = plt.subplot(111)

        self.q1_s = -160
        self.q1_end = 160
        self.q2_s = -160
        self.q2_end = 160
        self.q3_s = -160
        self.q3_end = 160
        self.q4_s = -160
        self.q4_end = 160
        self.q5_s = -160
        self.q5_end = 160
        self.q6_s = -160
        self.q6_end = 160
        N = 1000
        theta1 = self.q1_end + (self.q1_end - self.q1_s) * np.random.rand(N, 1)
        theta2 = self.q2_end + (self.q2_end - self.q2_s) * np.random.rand(N, 1)
        theta3 = self.q3_end + (self.q3_end - self.q3_s) * np.random.rand(N, 1)
        theta4 = self.q4_end + (self.q4_end - self.q4_s) * np.random.rand(N, 1)
        theta5 = self.q5_end + (self.q5_end - self.q5_s) * np.random.rand(N, 1)
        theta6 = self.q6_end + (self.q6_end - self.q6_s) * np.random.rand(N, 1)

        for i in range(N):
            q1 = theta1[i, 0]
            q2 = theta2[i, 0]
            q3 = theta3[i, 0]
            q4 = theta4[i, 0]
            q5 = theta5[i, 0]
            q6 = theta6[i, 0]
            self.T = self.robot.fkine(
                [q1 * du, q2 * du, q3 * du, q4 * du, q5 * du, q6 * du]
            )
            t = np.round(self.T.t, 3)
            r = np.round(self.T.rpy('deg'), 3)
            self.T_x.append(t[0])
            self.T_y.append(t[1])
            self.T_z.append(t[2])
            self.T_roll.append(int(r[0]))
            self.T_pitch.append(int(r[1]))
            self.T_yaw.append(int(r[2]))
            i = i + 1
        rospy.loginfo("Through the Work space in the interface to calculate of the robot.")
        
    def random_select_point(self):
        excel_file = Workbook()
        sheet = excel_file.active

        for i in range(10):
            x = np.random.randint(1,999)
            sheet.cell(row=i + 1, column=1).value = self.T_x[x]
            sheet.cell(row=i + 1, column=2).value = self.T_y[x]
            sheet.cell(row=i + 1, column=3).value = self.T_z[x]
            sheet.cell(row=i + 1, column=4).value = self.T_roll[x]
            sheet.cell(row=i + 1, column=5).value = self.T_pitch[x]
            sheet.cell(row=i + 1, column=6).value = self.T_yaw[x]

        file_name = self.xlsx_outpath + "/task_point" +".xlsx"
        excel_file.save(file_name)
        # rospy.loginfo("task point excel write down.")
        # print("================================")
        


if __name__ == '__main__':
    env = RobotOptEnv()
    
    action = env.action_space.sample()
    print(action)
    print(action[0])
    print(type(action[0]))
    print(action[1])
    print(action[2])
    
