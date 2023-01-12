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
        self.res = self.motor.TECO_member
        self.high_torque = 120.0 # 預設標準值 馬達極限 120.0 N max
        self.low_torque = 60.0 # 預設標準值 馬達極限 60.0 N rated
        self.motor_cost_init = np.array([200,200,200,200,200,200], dtype=np.float32) # 預設最大馬達費用
        self.motor_weight_init = np.array([2.0,2.0,2.0,2.0,2.0,2.0], dtype=np.float32) # 預設最大馬達重量
        self.motor_cost = np.array([200,200,200,200,200,200], dtype=np.float32) # 預設最大馬達費用
        self.motor_weight = np.array([2.0,2.0,2.0,2.0,2.0,2.0], dtype=np.float32) # 預設最大馬達重量
        self.motor_rated = np.array([198,198,198,198,198,198], dtype=np.float32)
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
        self.action_space = spaces.Discrete(16)
        
        # TODO: observation space for torque, reach, motor cost, weight, manipulability
        self.observation_space = spaces.Box(np.array([self.low_torque,self.low_torque,self.low_torque,self.low_torque,self.low_torque,self.low_torque, self.low_reach_eva, -float('inf'), -float('inf'), self.low_manipulability ]), 
                                            np.array([self.high_torque,self.high_torque,self.high_torque,self.high_torque,self.high_torque,self.high_torque, self.high_reach_eva, self.total_cost, self.total_weight, self.high_manipulability]), 
                                            dtype=np.float32)
        # TODO: reward 歸一化
        self.state = np.array([0,0,0,0,0,0,0,0,0,0], dtype=np.float32)
        self.pre_state = np.array([0,0,0,0,0,0,0,0,0,0], dtype=np.float32)

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
        
    # def step(self, action):
    #     assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))
    #     self.pre_state[0:6] = self.state[0:6]
    #     # TODO: 向量編碼動作
    #     if action == 0: # 軸2  # 短 # 型號1
    #         self.std_L2 -= 1.0
    #         # 配置軸2 motor 型號1
    #         motor_type = 0 # 型號
    #         axis = 2 # 軸數
            
    #     elif action == 1: # 軸2  # 長 # 型號1
    #         self.std_L2 += 1.0
    #         motor_type = 0 # 型號
    #         axis = 2 # 軸數
            
    #     elif action == 2: # 軸2  # 短 # 型號2
    #         self.std_L2 -= 1.0
    #         motor_type = 1 # 型號
    #         axis = 2 # 軸數
            
    #     elif action == 3: # 軸2  # 長 # 型號2
    #         self.std_L2 += 1.0
    #         motor_type = 1 # 型號
    #         axis = 2 # 軸數
            
    #     elif action == 4: # 軸2  # 短 # 型號3
    #         self.std_L2 -= 1.0
    #         motor_type = 2 # 型號
    #         axis = 2 # 軸數
            
    #     elif action == 5: # 軸2  # 長 # 型號3
    #         self.std_L2 += 1.0
    #         motor_type = 2 # 型號
    #         axis = 2 # 軸數
            
    #     elif action == 6: # 軸2  # 短 # 型號4
    #         self.std_L2 -= 1.0
    #         motor_type = 3 # 型號
    #         axis = 2 # 軸數
            
    #     elif action == 7: # 軸2  # 長 # 型號4
    #         self.std_L2 += 1.0
    #         motor_type = 3 # 型號
    #         axis = 2 # 軸數
            
    #     elif action == 8: # 軸3  # 短 # 型號1
    #         self.std_L3 -= 1.0
    #         motor_type = 0 # 型號
    #         axis = 3 # 軸數
            
    #     elif action == 9: # 軸3  # 長 # 型號1
    #         self.std_L3 += 1.0
    #         motor_type = 0 # 型號
    #         axis = 3 # 軸數
            
    #     elif action == 10: # 軸3  # 短 # 型號2
    #         self.std_L3 -= 1.0
    #         motor_type = 1 # 型號
    #         axis = 3 # 軸數
            
    #     elif action == 11: # 軸3  # 長 # 型號2
    #         self.std_L3 += 1.0
    #         motor_type = 1 # 型號
    #         axis = 3 # 軸數
            
    #     elif action == 12: # 軸3  # 短 # 型號3
    #         self.std_L3 -= 1.0
    #         motor_type = 2 # 型號
    #         axis = 3 # 軸數
            
    #     elif action == 13: # 軸3  # 長 # 型號3
    #         self.std_L3 += 1.0
    #         motor_type = 2 # 型號
    #         axis = 3 # 軸數
            
    #     elif action == 14: # 軸3  # 短 # 型號4
    #         self.std_L3 -= 1.0
    #         motor_type = 3 # 型號
    #         axis = 3 # 軸數
            
    #     elif action == 15: # 軸3  # 長 # 型號4
    #         self.std_L3 += 1.0
    #         motor_type = 3 # 型號
    #         axis = 3 # 軸數
            
        
        
    #     # 輸入action後 二,三軸軸長
    #     self.robot_urdf.specified_generate_write_urdf(self.std_L2, self.std_L3)
    #     self.robot.__init__() # 重製機器人
    #     torque = self.dynamics_torque_limit()
    #     self.state[0:6] = torque
        
    #     # TODO: 改為可達性分析
    #     reach_score = self.reach_evaluate()
    #     manipulability_score = self.manipulability_evaluate()
    #     rospy.loginfo("reach_score: %s", reach_score)
    #     rospy.loginfo("manipulability_score: %s", manipulability_score)
    #     # 計算最大可達半徑
    #     L1,L2,L3 = self.robot.return_configuration()
    #     L_sum = L1+L2+L3
    #     rospy.loginfo("configuration axis length: %s, %s, %s, %s", L1, L2, L3, L_sum)
    #     self.state[6] = L_sum
    #     self.counts += 1
        
    #     # 計算成本與重量    
    #     self.motor_cost[axis-1] = self.res.cost[motor_type]
    #     self.motor_weight[axis-1] = self.res.weight[motor_type]
    #     cost = sum(self.motor_cost) 
    #     weight = sum(self.motor_weight)
    #     self.state[7] = cost
    #     self.state[8] = weight
    #     self.motor_rated[axis-1] = self.res.rated_torque[motor_type]
    #     rospy.loginfo("configuration cost & weight: %s, %s", cost, weight)
        
    #     # if down 完成任务 
        
    #     torque_reward_close = 0.0
        
    #     for i in range(6):
    #         # TODO:consider cost & weight 
    #         if np.abs(self.state[i]) > self.motor_rated[i]:
    #             self.torque_done[i] = True # 已經完成任務
    #         else:
    #             if np.abs(self.pre_state[i]) > np.abs(self.state[i]): # 如果前一個狀態比這一個狀態torque大
    #                 torque_reward_close = torque_reward_close + 2.0
    #             else:
    #                 torque_reward_close = torque_reward_close - 2.0
    #             self.torque_done[i] = False

    #     if cost< self.pre_state[7]: # 成本比上一次低
    #         torque_reward_close = torque_reward_close + 1.0
    #     else:
    #         torque_reward_close = torque_reward_close - 1.0
    #     if weight< self.pre_state[8]: # 重量比上一次低
    #         torque_reward_close = torque_reward_close + 1.0
    #     else:
    #         torque_reward_close = torque_reward_close - 1.0
            
    #     # 如果所有軸都超過最大torque，則結束
    #     false_done = False in self.torque_done 
    #     # 終止條件: 可達半徑低於使用者設定 or 超出各軸馬達最大torque 範圍 , 整體手臂重量高於使用者設定, 累計步數大於200次
    #     if L_sum < self.reach_distance or cost > self.total_cost or weight > self.total_weight:
    #         false_done = False
            
    #     # 走一步修正, 但還未最佳化完成
    #     if false_done:
    #         reward = torque_reward_close
    #         # The episode truncates at 30 time steps.
    #         if self.counts == 200:
    #             reward = 30.0
    #             false_done = False
                
    #     # down 完成後, 定義所計算出的torque值, 分數加多少
    #     else:
    #         if L_sum < self.reach_distance:
    #             reward = -20.0
    #         elif cost > self.total_cost:
    #             reward = -10.0
    #         elif weight > self.total_weight:
    #             reward = -10.0
    #         else:
    #             reward = 0.0
    #             for i in range(6):
    #                 # 即torque, 超過最大torque 
    #                 if np.abs(self.state[i]) > self.motor_rated[i]:
    #                     reward += -5.0
    #                 else:
    #                     reward += 0.0
    #                 # TODO: 新增可操作性評估
    #                     # manipulability_score = self.manipulability_evaluate()
    #     done = not false_done # 取bool 反向值
        
    #     return self.state, reward, done, {}
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
            
        elif action == 6: # 軸2  # 短 # 型號4
            self.std_L2 -= 1.0
            motor_type = 3 # 型號
            axis = 2 # 軸數
            
        elif action == 7: # 軸2  # 長 # 型號4
            self.std_L2 += 1.0
            motor_type = 3 # 型號
            axis = 2 # 軸數
            
        elif action == 8: # 軸3  # 短 # 型號1
            self.std_L3 -= 1.0
            motor_type = 0 # 型號
            axis = 3 # 軸數
            
        elif action == 9: # 軸3  # 長 # 型號1
            self.std_L3 += 1.0
            motor_type = 0 # 型號
            axis = 3 # 軸數
            
        elif action == 10: # 軸3  # 短 # 型號2
            self.std_L3 -= 1.0
            motor_type = 1 # 型號
            axis = 3 # 軸數
            
        elif action == 11: # 軸3  # 長 # 型號2
            self.std_L3 += 1.0
            motor_type = 1 # 型號
            axis = 3 # 軸數
            
        elif action == 12: # 軸3  # 短 # 型號3
            self.std_L3 -= 1.0
            motor_type = 2 # 型號
            axis = 3 # 軸數
            
        elif action == 13: # 軸3  # 長 # 型號3
            self.std_L3 += 1.0
            motor_type = 2 # 型號
            axis = 3 # 軸數
            
        elif action == 14: # 軸3  # 短 # 型號4
            self.std_L3 -= 1.0
            motor_type = 3 # 型號
            axis = 3 # 軸數
            
        elif action == 15: # 軸3  # 長 # 型號4
            self.std_L3 += 1.0
            motor_type = 3 # 型號
            axis = 3 # 軸數
            
        
        
        # 輸入action後 二,三軸軸長
        self.robot_urdf.specified_generate_write_urdf(self.std_L2, self.std_L3)
        self.robot.__init__() # 重製機器人
        torque = self.dynamics_torque_limit()
        self.state[0:6] = torque
        # 可達性
        self.state[6] = self.reach_evaluate()
        # 計算成本與重量    
        self.motor_cost[axis-1] = self.res.cost[motor_type]
        self.motor_weight[axis-1] = self.res.weight[motor_type]
        cost = sum(self.motor_cost) 
        weight = sum(self.motor_weight)
        self.state[7] = cost
        self.state[8] = weight
        # 可操作性
        self.state[9] = self.manipulability_evaluate()
        # rospy.loginfo("reach_score: %s", self.state[6])
        # rospy.loginfo("manipulability_score: %s", self.state[9])
        self.motor_rated[axis-1] = self.res.rated_torque[motor_type]
        # rospy.loginfo("configuration cost & weight: %s, %s", cost, weight)
        self.counts += 1
        
        reward = 0
        shaping = (
            -10 * np.sqrt(self.state[0] * self.state[0] + self.state[1] * self.state[1] + self.state[2] * self.state[2] + self.state[3] * self.state[3] + self.state[4] * self.state[4] + self.state[5] * self.state[5])
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
            if np.abs(self.state[i]) > 114: # TODO: fixed 
                self.torque_over = True # 超出最大扭矩
                break # TODO

        terminated = False
        # if down 完成任务 
        if self.state[6] == 0: # TODO: fixed
            terminated = True
            reward = -50
        if self.torque_over == True: # TODO: fixed 0112 00:22 改為超過最大的馬達型號torque
            terminated = True
            reward = -100
        # if self.torque_over == False and self.state[6] != 0 and self.state[8] < self.op_weight and self.state[7] < self.op_cost:  # TODO: fixed 增加 cost & weight & ... 
        #     terminated = True
        #     reward = +100
        if self.counts == 30:
            terminated = True
            self.counts = 0
            reward = +30
        self.torque_over = False #reset
        rospy.loginfo("counts: %s", self.counts)
        rospy.loginfo("step_reward: %s", reward)
        return self.state, reward, terminated, {}
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
        reach_score = self.reach_evaluate()
        manipulability_score = self.manipulability_evaluate()
        self.state[6] = reach_score
        self.state[7] = sum(self.motor_cost_init)
        self.state[8] = sum(self.motor_weight_init)
        self.state[9] = manipulability_score
        # rospy.loginfo("configuration: %s, %s, %s, %s", L1, L2, L3, L_sum)
        rospy.loginfo("self.state: %s", self.state)
        
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
    def reach_evaluate(self):
        # import xlsx
        df = load_workbook("./xlsx/task_point.xlsx")
        sheets = df.worksheets
        sheet1 = sheets[0]
        rows = sheet1.rows
        cols = sheet1.columns
        T_tmp = []
        score = []
        i = 0
        false_done = False
        for row in rows:
            row_val = [col.value for col in row]
            T_tmp.append(SE3(row_val[0], row_val[1], row_val[2]) * SE3.RPY([np.deg2rad(row_val[3]), np.deg2rad(row_val[4]), np.deg2rad(row_val[5])]))
            ik_q = self.robot.ikine_LM(T=T_tmp[i])
            # 先判斷逆運動學解是否存在(即平方殘差是否低於1e-12), 當有不存在則評估0分
            if ik_q.success == True:
                # 迭代500次以內的迭代次數為評估
                iterations_count = 500 - ik_q.iterations
                score.append(iterations_count/500)
            else:
                # score[i] = 0
                false_done = True
                break
            i = i + 1
        if false_done == True:
            final_score = 0
            print("FFFFFFFFFFFFFUUUUUUUUUCCCCCCCCCCCKKKKKKK")
        else:
            final_score = np.mean(score)

        return(final_score) # 回傳 

    # TODO: 新增可操作性評估
    def manipulability_evaluate(self):
        # import xlsx
        df = load_workbook("./xlsx/task_point.xlsx")
        sheets = df.worksheets
        sheet1 = sheets[0]
        rows = sheet1.rows
        cols = sheet1.columns

        T_tmp = []
        manipulability_index = []
        i = 0
        for row in rows:
            row_val = [col.value for col in row]
            T_tmp.append(SE3(row_val[0], row_val[1], row_val[2]) * SE3.RPY([np.deg2rad(row_val[3]), np.deg2rad(row_val[4]), np.deg2rad(row_val[5])]))
            # print(T_tmp[i])
            ik_q = self.robot.ikine_LM(T=T_tmp[i])
            if ik_q.success == True:
                manipulability_index.append(self.robot.manipulability(q=ik_q.q))
                # robot.plot_ellipse()
                ik_np = np.array(ik_q.q)
                # print(ik_np)
                # self.robot.plot(q=ik_np, backend='pyplot', dt = 0.5)
            i = i + 1

        return(np.mean(manipulability_index)) # 回傳 manipulability 取平均

if __name__ == '__main__':
    env = RobotOptEnv()
    
    action = env.action_space.sample()
    print(action)
    print(action[0])
    print(type(action[0]))
    print(action[1])
    print(action[2])
    
