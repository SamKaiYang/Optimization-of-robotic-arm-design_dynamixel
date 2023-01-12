#!/usr/bin/env python3
# coding: utf-8
"""
View more, visit my tutorial page: https://mofanpy.com/tutorials/
My Youtube Channel: https://www.youtube.com/user/MorvanZhou
More about Reinforcement learning: https://mofanpy.com/tutorials/machine-learning/reinforcement-learning/
Dependencies:
torch: 0.4
gym: 0.8.1
numpy
"""
import rospy
import sys
# dqn 
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import gym

# msg 
from interface_control.msg import cal_cmd, dyna_data, dyna_space_data, cal_process
from interface_control.msg import cal_result, specified_parameter_design, optimal_design, optimal_random
import geometry_msgs.msg

import math
from math import pi

from random_robot import RandomRobot
from motor_module import mootor_data
import pandas as pd


# Hyper Parameters
BATCH_SIZE = 32
LR = 0.01                   # learning rate
EPSILON = 0.9               # greedy policy
GAMMA = 0.9                 # reward discount
TARGET_REPLACE_ITER = 100   # target update frequency
MEMORY_CAPACITY = 2000
env = gym.make('CartPole-v0')
env = env.unwrapped
N_ACTIONS = env.action_space.n
N_STATES = env.observation_space.shape[0]
ENV_A_SHAPE = 0 if isinstance(env.action_space.sample(), int) else env.action_space.sample().shape     # to confirm the shape


class Net(nn.Module):
    def __init__(self, ):
        super(Net, self).__init__()
        self.fc1 = nn.Linear(N_STATES, 50)
        self.fc1.weight.data.normal_(0, 0.1)   # initialization
        self.out = nn.Linear(50, N_ACTIONS)
        self.out.weight.data.normal_(0, 0.1)   # initialization

    def forward(self, x):
        x = self.fc1(x)
        x = F.relu(x)
        actions_value = self.out(x)
        return actions_value


class DQN(object):
    def __init__(self):
        self.eval_net, self.target_net = Net(), Net()

        self.learn_step_counter = 0                                     # for target updating
        self.memory_counter = 0                                         # for storing memory
        self.memory = np.zeros((MEMORY_CAPACITY, N_STATES * 2 + 2))     # initialize memory
        self.optimizer = torch.optim.Adam(self.eval_net.parameters(), lr=LR)
        self.loss_func = nn.MSELoss()

        # dynamics optimal design Parameters
        # callback:Enter the parameters of the algorithm to be optimized on the interface
        self.sub_optimal_design = rospy.Subscriber("/optimal_design",optimal_design, self.optimal_design_callback)
        # callback:Randomly generated shaft length due to optimization algorithm
        self.sub_optimal_random = rospy.Subscriber("/optimal_random",optimal_random, self.optimal_random_callback)
        
    def choose_action(self, x):
        x = torch.unsqueeze(torch.FloatTensor(x), 0)
        # input only one sample
        if np.random.uniform() < EPSILON:   # greedy
            actions_value = self.eval_net.forward(x)
            action = torch.max(actions_value, 1)[1].data.numpy()
            action = action[0] if ENV_A_SHAPE == 0 else action.reshape(ENV_A_SHAPE)  # return the argmax index
        else:   # random
            action = np.random.randint(0, N_ACTIONS)
            action = action if ENV_A_SHAPE == 0 else action.reshape(ENV_A_SHAPE)
        return action

    def store_transition(self, s, a, r, s_):
        transition = np.hstack((s, [a, r], s_))
        # replace the old memory with new memory
        index = self.memory_counter % MEMORY_CAPACITY
        self.memory[index, :] = transition
        self.memory_counter += 1

    def learn(self):
        # target parameter update
        if self.learn_step_counter % TARGET_REPLACE_ITER == 0:
            self.target_net.load_state_dict(self.eval_net.state_dict())
        self.learn_step_counter += 1

        # sample batch transitions
        sample_index = np.random.choice(MEMORY_CAPACITY, BATCH_SIZE)
        b_memory = self.memory[sample_index, :]
        b_s = torch.FloatTensor(b_memory[:, :N_STATES])
        b_a = torch.LongTensor(b_memory[:, N_STATES:N_STATES+1].astype(int))
        b_r = torch.FloatTensor(b_memory[:, N_STATES+1:N_STATES+2])
        b_s_ = torch.FloatTensor(b_memory[:, -N_STATES:])

        # q_eval w.r.t the action in experience
        q_eval = self.eval_net(b_s).gather(1, b_a)  # shape (batch, 1)
        q_next = self.target_net(b_s_).detach()     # detach from graph, don't backpropagate
        q_target = b_r + GAMMA * q_next.max(1)[0].view(BATCH_SIZE, 1)   # shape (batch, 1)
        loss = self.loss_func(q_eval, q_target)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

    # env data callback
    def optimal_design_callback(self, data):
        # print(data.data)
        self.op_payload = data.payload
        self.op_payload_position = data.payload_position
        self.op_vel = data.vel
        self.op_acc = data.acc
        self.op_radius = data.radius

        rospy.loginfo("I heard op_payload is %s", self.op_payload)
        rospy.loginfo("I heard op_payload_position is %s", self.op_payload_position)
        rospy.loginfo("I heard op_vel is %s", self.op_vel)
        rospy.loginfo("I heard op_acc is %s", self.op_acc)
        rospy.loginfo("I heard op_radius is %s", self.op_radius)

        # print(self.optimal_design_flag)

    # random generated data callback
    def optimal_random_callback(self, data):
        self.op_axis_2_length = data.axis_2_length
        self.op_axis_3_length = data.axis_3_length

        rospy.loginfo("I heard op_axis_2_length is %s", self.op_axis_2_length)
        rospy.loginfo("I heard op_axis_3_length is %s", self.op_axis_3_length)

    # TODO: optimization_algorithm: use Random forest
    def optimization_algorithm(self):
        # input data: random axis2,3 length, robot workspace, robot payload, robot joint velocity, robot joint acceleration, motor data
        '''
        Agent :
            robot payload set
            robot velocity
            robot acceleration

        Action :
            axis 2 length increase
            axis 2 length reduce
            axis 3 length increase
            axis 3 length reduce
            Change the motor configuration of each axis

        Rewards :
            torque
            motor cost
            robot workspace
            robot weight

        Status :
            After the parameter of action is changed, the torque value of each axis
        '''

        ''' transfer the data to the dataframe
        agent:
            self.op_payload
            self.op_payload_position
            self.op_vel
            self.op_acc
            self.op_radius
        '''
        # TODO: axis2,3 length change
        ''' receive the data from the topic
        action:
            axis 2 length increase
            axis 2 length reduce
            axis 3 length increase
            axis 3 length reduce
            Change the motor configuration of each axis
        '''
        # TODO: rebuild robot
        self.robot_motor_random_build()
        # update dynamics torque calculation parameters
        self.payload = self.op_payload
        self.payload_position = self.op_payload_position
        self.vel = self.op_vel
        self.acc = self.op_acc
        # calculate the robotic arm workspace
        self.Workspace_cal_Monte_Carlo()
        # Compare ideal radius with the workspace radius
        print("T_x:",self.T_x[0,:].max()-self.T_x[0,:].min())
        print("T_y:",self.T_y[0,:].max()-self.T_y[0,:].min())
        print("T_z:",self.T_z[0,:].max()-self.T_z[0,:].min())
        radius_max = self.T_x[0,:].max()-self.T_x[0,:].min()
        radius_reward = self.op_radius - radius_max
        # TODO: before reward
        # output data: robot torque, robot module, motor select
        # TODO: use dynamics to calculate torque
        self.dynamics_torque_limit()
        ''' transfer the data to the topic
        rewards:
            torque : Use torque reduction
            motor cost
            robot workspace
            robot weight
        '''
        # Use torque reduction, motor score (the higher the cost, the lower the score),
        # Motor score (the higher the cost, the lower the score)
        # Scope of work (the larger the scope of work, the higher the score)
        ''' transfer the data to the topic
        state:
        After the parameter of action is changed, the torque value of each axis
        '''
        # TODO: through optimization algorithm to find the best solution

dqn = DQN()

print('\nCollecting experience...')
for i_episode in range(400):
    s = env.reset()
    ep_r = 0
    while True:
        env.render()
        a = dqn.choose_action(s)

        # take action
        s_, r, done, info = env.step(a)

        # modify the reward
        x, x_dot, theta, theta_dot = s_
        r1 = (env.x_threshold - abs(x)) / env.x_threshold - 0.8
        r2 = (env.theta_threshold_radians - abs(theta)) / env.theta_threshold_radians - 0.5
        r = r1 + r2

        dqn.store_transition(s, a, r, s_)

        ep_r += r
        if dqn.memory_counter > MEMORY_CAPACITY:
            dqn.learn()
            if done:
                print('Ep: ', i_episode,
                        '| Ep_r: ', round(ep_r, 2))

        if done:
            break
        s = s_