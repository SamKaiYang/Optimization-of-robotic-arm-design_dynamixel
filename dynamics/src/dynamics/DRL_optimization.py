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
from interface_control.msg import (cal_cmd, cal_process, cal_result, dyna_data,
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
from dqn import DQN

np.set_printoptions(
    linewidth=100,
    formatter={"float": lambda x: f"{x:8.4g}" if abs(x) > 1e-10 else f"{0:8.4g}"},
)

from arm_workspace import arm_workspace_plane
# from robot_urdf import RandomRobot
from motor_module import motor_data
from random_robot import RandomRobot
from modular_robot_6dof import modular_robot_6dof
# DRL_optimization api
import sys
import os
import torch.nn as nn
import torch.nn.functional as F
curr_path = os.path.dirname(os.path.abspath(__file__))  # 当前文件所在绝对路径
parent_path = os.path.dirname(curr_path)  # 父路径
sys.path.append(parent_path)  # 添加路径到系统路径

import gym
import torch
import datetime
import numpy as np
from common.utils import save_results, make_dir
from common.utils import plot_rewards
from dqn import DQN

import matplotlib.pyplot as plt
from RobotOptEnv_dynamixel import RobotOptEnv
import tensorboardX
import yaml
file_path = curr_path + "/outputs/" 
curr_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")  # 获取当前时间

tb = tensorboardX.SummaryWriter()
# tensorboard_callback = tensorboardX.(log_dir=log_dir, histogram_freq=1)
algo_name = "DQN"  # 算法名称
env_name = 'RobotOptEnv'  # 环境名称

class MLP(nn.Module):
    def __init__(self, n_states,n_actions,hidden_dim=128):
        """ 初始化q网络，为全连接网络
            n_states: 输入的特征数即环境的状态维度
            n_actions: 输出的动作维度
        """
        super(MLP, self).__init__()
        self.fc1 = nn.Linear(n_states, hidden_dim) # 输入层
        self.fc2 = nn.Linear(hidden_dim,hidden_dim) # 隐藏层
        self.fc3 = nn.Linear(hidden_dim, n_actions) # 输出层
        
    def forward(self, x):
        # 各层对应的激活函数
        x = F.relu(self.fc1(x)) 
        x = F.relu(self.fc2(x))
        return self.fc3(x)

class drl_optimization:
    def __init__(self):
        # self.test = 0
        self.robot = modular_robot_6dof()
        self.env = RobotOptEnv()

    def env_agent_config(self, cfg, seed=1):
        ''' 创建环境和智能体
        '''
        # env = gym.make(self.env)  # 创建环境
        env = self.env  # 创建环境
        env.seed(seed)  # 设置随机种子
        n_states = env.observation_space.shape[0]  # 状态维度
        rospy.loginfo("n_states: {}".format(n_states))
        n_actions = env.action_space.n  # 动作數量
        rospy.loginfo("n_actions: {}".format(n_actions))
        model = MLP(n_states,n_actions)
        agent = DQN(n_actions,model,cfg)  # 创建智能体
        return env, agent
    
    # dqn train
    def train(self, cfg, env, agent):
        ''' 訓練
        '''
        print('開始訓練!')
        print(f'環境：{cfg.env_name}, 算法：{cfg.algo_name},設備：{cfg.device}')
        rewards = [] # 记录所有回合的奖励
        ma_rewards = []  # 记录所有回合的滑动平均奖励
        for i_ep in range(cfg.train_eps):
            if rospy.is_shutdown():
                break
            ep_reward = 0 # 记录一回合内的奖励
            state = env.reset() # 重置环境，返回初始状态
            # rospy.loginfo("state: {}".format(state))
            # while True:
            # rospy.loginfo("next train eps: {}".format(i_ep))
            while not rospy.is_shutdown():
                rospy.loginfo("=============================")
                action = agent.choose_action(state) # 选择动作
                rospy.loginfo("action: {}".format(action))
                next_state, reward, done, _ = env.step(action) # 更新环境，返回transition
                # rospy.loginfo("next_state: {}".format(next_state))
                # rospy.loginfo("reward: {}".format(reward))
                # rospy.loginfo("done: {}".format(done))

                agent.memory.push(state, action, reward, next_state, done) # 保存transition
                state = next_state # 更新下一个状态
                rospy.loginfo("state: {}".format(state))
                agent.update() # 更新智能体
                ep_reward += reward # 累加奖励
                rospy.loginfo("ep_reward: {}".format(ep_reward))
                if done:
                    break
            if (i_ep+1) % cfg.target_update == 0: # 智能体目标网络更新
                agent.target_net.load_state_dict(agent.policy_net.state_dict())
            rewards.append(ep_reward)
            if ma_rewards:
                ma_rewards.append(0.9*ma_rewards[-1]+0.1*ep_reward)
            else:
                ma_rewards.append(ep_reward)
            # if (i_ep+1)%5 == 0: 
            print('回合：{}/{}, 獎勵：{}'.format(i_ep+1, cfg.train_eps, ep_reward))
            tb.add_scalar("/trained-model/log/", ep_reward, i_ep+1)
        print('完成训练！')
        # export scalar data to JSON for external processing
        # tb.export_scalars_to_json("./all_scalars.json")
        # tb.close()
        return rewards, ma_rewards

    def test(self, cfg,env,agent):
        print('開始測試!')
        print(f'環境：{cfg.env_name}, 算法：{cfg.algo_name}, 設備：{cfg.device}')
        # 由于测试不需要使用epsilon-greedy策略，所以相应的值设置为0
        cfg.epsilon_start = 0.0 # e-greedy策略中初始epsilon
        cfg.epsilon_end = 0.0 # e-greedy策略中的终止epsilon
        rewards = [] # 记录所有回合的奖励
        ma_rewards = []  # 记录所有回合的滑动平均奖励
        for i_ep in range(cfg.test_eps):
            if rospy.is_shutdown():
                break
            ep_reward = 0 # 记录一回合内的奖励
            state = env.reset() # 重置环境，返回初始状态
            # while True:
            while not rospy.is_shutdown():
                action = agent.choose_action(state) # 选择动作
                next_state, reward, done, _ = env.step(action) # 更新环境，返回transition
                state = next_state # 更新下一个状态
                ep_reward += reward # 累加奖励
                
                if done:
                    break
            rewards.append(ep_reward)
            if ma_rewards:
                ma_rewards.append(ma_rewards[-1]*0.9+ep_reward*0.1)
            else:
                ma_rewards.append(ep_reward)
            print(f"回合：{i_ep+1}/{cfg.test_eps}，獎勵：{ep_reward:.1f}")
            tb.add_scalar("/tested-model/log/", ep_reward, i_ep+1)
        print('完成測試！')
        return rewards,ma_rewards

class DQNConfig:
    ''' 算法相关参数设置
    '''

    def __init__(self):
        self.algo_name = algo_name  # 算法名称
        self.env_name = env_name  # 环境名称
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")  # 检测GPU
        self.train_eps = 300  # 训练的回合数
        self.test_eps = 20  # 测试的回合数
        # 超参数
        self.gamma = 0.99  # 强化学习中的折扣因子
        self.epsilon_start = 0.99  # e-greedy策略中初始epsilon
        self.epsilon_end = 0.005  # e-greedy策略中的终止epsilon
        self.epsilon_decay = 500  # e-greedy策略中epsilon的衰减率
        self.lr = 0.0001  # 学习率
        self.memory_capacity = 100000  # 经验回放的容量
        self.batch_size = 128  # mini-batch SGD中的批量大小
        self.target_update = 4  # 目标网络的更新频率
        self.hidden_dim = 512  # 网络隐藏层
class PlotConfig:
    ''' 绘图相关参数设置
    '''

    def __init__(self) -> None:
        self.algo_name = algo_name  # 算法名称
        self.env_name = env_name  # 环境名称
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")  # 检测GPU
        self.result_path = curr_path + "/outputs/" + self.env_name + \
            '/' + curr_time + '/results/'  # 保存结果的路径
        self.model_path = curr_path + "/outputs/" + self.env_name + \
            '/' + curr_time + '/models/'  # 保存模型的路径
        self.save = True  # 是否保存图片

class RosTopic:
    def __init__(self):
        self.sub_taskcmd = rospy.Subscriber("/cal_command", cal_cmd, self.cmd_callback)
        self.cmd_run = 0
    def cmd_callback(self, data):
        self.cmd = data.cmd
        rospy.loginfo("I heard command is %s", data.cmd)
        if data.cmd == 22:
            rospy.sleep(10)
            self.cmd_run = 1
            
if __name__ == "__main__":
    rospy.init_node("optimization")
    a = 0
    cfg = DQNConfig()
    drl = drl_optimization()
    plot_cfg = PlotConfig()
    ros_topic = RosTopic()
    while not rospy.is_shutdown():
        if ros_topic.cmd_run == 1:
            ros_topic.cmd_run = 0
            # 訓練
            train_env, train_agent = drl.env_agent_config(cfg, seed=1)
            train_rewards, train_ma_rewards = drl.train(cfg, train_env, train_agent)
            make_dir(plot_cfg.result_path, plot_cfg.model_path)  # 创建保存结果和模型路径的文件夹
            train_agent.save(path=plot_cfg.model_path)  # 保存模型
            save_results(train_rewards, train_ma_rewards, tag='train',
                        path=plot_cfg.result_path)  # 保存结果
            plot_rewards(train_rewards, train_ma_rewards, plot_cfg, tag="train")  # 画出结果
            # 測試
            test_env, test_agent = drl.env_agent_config(cfg, seed=10)
            test_agent.load(path=plot_cfg.model_path)  # 导入模型
            test_rewards, test_ma_rewards = drl.test(cfg, test_env, test_agent)
            save_results(test_rewards, test_ma_rewards, tag='test',
                        path=plot_cfg.result_path)  # 保存结果
            plot_rewards(test_rewards, test_ma_rewards, plot_cfg, tag="test")  # 画出结果
            break
        else:
            pass