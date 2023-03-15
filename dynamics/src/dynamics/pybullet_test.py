#!/usr/bin/env python3
# coding: utf-8
import importlib
import rospy
import sys
importlib.reload(sys)
import os
import numpy as np
curr_path = os.path.dirname(os.path.abspath(__file__))  # 当前文件所在绝对路径
parent_path = os.path.dirname(curr_path)  # 父路径
sys.path.append(parent_path)  # 添加路径到系统路径


from stl_conv_6dof_urdf_dynamixel import stl_conv_urdf
from RobotOptEnv_dynamixel_v2 import RobotOptEnv

import pybullet as p
import time
import pybullet_data
import os
from termcolor import cprint

from pybullet_planning import BASE_LINK, RED, BLUE, GREEN
from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui, set_camera, wait_for_duration, wait_if_gui, apply_alpha
from pybullet_planning import Pose, Point, Euler
from pybullet_planning import multiply, invert, get_distance
from pybullet_planning import create_obj, create_attachment, Attachment
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name, get_disabled_collisions, \
    get_body_body_disabled_collisions, has_link, are_links_adjacent
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn,get_extend_fn, plan_joint_motion, get_difference_fn,get_collision_fn
from pybullet_planning import dump_world, set_pose
from pybullet_planning import get_collision_fn, get_floating_body_collision_fn, expand_links, create_box, create_sphere
from pybullet_planning import pairwise_collision, pairwise_collision_info, draw_collision_diagnosis, body_collision_info
from pybullet_planning import rrt_connect
import random
import math
import trimesh
# import dynamics.RobotOptEnv_dynamixel_v2
from spatialmath import SE3
import spatialmath as sm
from modular_robot_6dof import modular_robot_6dof

HERE = os.path.dirname(__file__)
SINGLE_ARM = os.path.join(HERE,'urdf', 'single_arm_v12_motion.urdf')

class motion_model(object):
    def __init__(self):
        self.length_1 = 0.1
        self.length_2 = 0.1
        self.length_3 = 0.1
    def random_exclude_range(self, start, end, exclude_start, exclude_end):
        while True:
            random_num = random.uniform(start, end)
            if not exclude_start <= random_num <= exclude_end:
                return random_num
    def random_obstacle(self):
        # create a box to be picked up
        # see: https://pybullet-planning.readthedocs.io/en/latest/reference/generated/pybullet_planning.interfaces.env_manager.create_box.html#pybullet_planning.interfaces.env_manager.create_box
        '''0.06	-0.17	-0.45
        -0.36	-0.06	0.40
        0.09	-0.32	-0.36
        0.06	-0.15	0.22
        -0.04	-0.15	1.02
        0.08	0.05	0.21
        0.02	-0.12	-0.25
        0.29	-0.19	0.72
        -0.06	-0.45	0.52
        0.00	-0.04	0.38'''
        
        # self.block = [create_sphere(0.01) for _ in range(10)]
        # # self.block = [create_box(0.03, 0.03, 0.03) for _ in range(10)]
        # block_positions = [ \
        # (0.06, -0.17, -0.45), (-0.36, -0.06, 0.40), (0.09, -0.32, -0.36), (0.06, -0.15, 0.22), \
        # (-0.04, -0.15, 1.02), (0.08, 0.05, 0.21), (0.02, -0.12, -0.25), (0.29, -0.19, 0.72), \
        # (-0.06, -0.45, 0.52), (0.00, -0.04, 0.38)]

        # for i, pos in enumerate(block_positions):
        #     set_pose(self.block[i], Pose(Point(x=pos[0], y=pos[1], z=pos[2]), Euler(yaw=np.pi/2)))
        #     p.addUserDebugText(str(block_positions[i]), block_positions[i], textColorRGB=[1, 0, 0], textSize=1)

        self.block_obstacles = [create_box(0.04, 0.04, 0.04) for _ in range(10)]
        # # Generate random spheres
        for i in range(10):
            radius = self.random_exclude_range(-0.5, 0.5, -0.2, 0.2)
            position = [self.random_exclude_range(-0.5, 0.5, -0.2, 0.2), self.random_exclude_range(-0.5, 0.5, -0.2, 0.2), radius]
            set_pose(self.block_obstacles[i], Pose(Point(x=position[0], y=position[1], z=position[2]), Euler(yaw=np.pi/2)))
            p.addUserDebugText(str(position), position, textColorRGB=[1, 0, 0], textSize=1)

    def motion_planning(self):
        pass
    def pybullet_init(self):
        pass

    def reset_robot_urdf(self, std_L2, std_L3):
        robot_urdf = stl_conv_urdf("single_arm_v12","test")
        robot_urdf.specified_generate_write_urdf(std_L2, std_L3)

    def stl_trimesh_scaling(self, std_L2_scale, std_L3_scale):
        # 加载包含三个组件的STL
        # if axis == 2:
        scale = std_L2_scale
        mesh = trimesh.load(HERE+'/meshes/original_single_arm_axis_2.STL')
        # mesh.show()
        # 获取需要缩放的组件和其他组件
        scaled_mesh = mesh.split()[1]  # 第二个组件

        # 0 2 7 8
        # base_meshes = mesh.split()[:1]
        base_meshes = trimesh.util.concatenate(mesh.split()[3]+mesh.split()[4]+mesh.split()[5]+mesh.split()[6])
        other_meshes = trimesh.util.concatenate(mesh.split()[0]+mesh.split()[2]+mesh.split()[7]+mesh.split()[8])
        
        # 将 STL 模型沿 X 轴方向偏移 1 厘米
        x_cm = scale - 12 # 沿 X 轴方向的偏移量（单位：厘米）
        x_m = x_cm / 100.0  # 将单位从厘米转换为米
        other_meshes.apply_translation([x_m, 0, 0])

        # 在X方向上缩放
        scale_factor = scale/12

        scaled_mesh.apply_scale([scale_factor, 1, 1])  # 只更改x軸的長度
        
        offset_x = -x_cm/2*0.01 # 将单位从厘米转换为米

        scaled_mesh.apply_translation([offset_x, 0, 0])
        # 将缩放后的组件和其他组件重新组合成一个网格对象
        new_mesh = trimesh.util.concatenate(base_meshes +scaled_mesh + other_meshes)

        # 保存为新的STL文件
        new_mesh.export(HERE+'/meshes/merged_single_arm_axis_2.STL')
        # new_mesh.show()
        # elif axis == 3:
        scale = std_L3_scale
        mesh = trimesh.load(HERE+'/meshes/original_single_arm_axis_3.STL')
        # mesh.show()
        # 获取需要缩放的组件和其他组件
        scaled_mesh = mesh.split()[3]  # 第四个组件

        # 0 2 7 8
        # base_meshes = mesh.split()[:1]
        base_meshes = trimesh.util.concatenate(mesh.split()[0]+mesh.split()[1]+mesh.split()[2]+mesh.split()[5])
        other_meshes = trimesh.util.concatenate(mesh.split()[4]+mesh.split()[6])
        
        # 将 STL 模型沿 X 轴方向偏移 1 厘米
        x_cm = scale - 12 # 沿 X 轴方向的偏移量（单位：厘米）
        x_m = x_cm / 100.0  # 将单位从厘米转换为米
        other_meshes.apply_translation([x_m, 0, 0])

        # 在X方向上缩放
        scale_factor = scale/12

        scaled_mesh.apply_scale([scale_factor, 1, 1])  # 只更改x軸的長度
        
        offset_x = -x_cm/2*0.01 # 将单位从厘米转换为米

        scaled_mesh.apply_translation([offset_x, 0, 0])
        # 将缩放后的组件和其他组件重新组合成一个网格对象
        new_mesh = trimesh.util.concatenate(base_meshes +scaled_mesh + other_meshes)

        # 保存为新的STL文件
        new_mesh.export(HERE+'/meshes/merged_single_arm_axis_3.STL')
        # new_mesh.show()

    def motion_planning_init(self, viewer=True):
        # viewer = True
        connect(use_gui=viewer)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # planeId = p.loadURDF("plane.urdf")
        
        robot_path = SINGLE_ARM

        self.Robot = load_pybullet(robot_path, fixed_base=True)
        arm='right'
        set_camera(yaw=0, pitch=-70, distance=2, target_position=(0, 0, 0))
        arm_tag = arm[0]
        arm_joint_names = ['Joint1',
                            'Joint2',
                            'Joint3',
                            'Joint4',
                            'Joint5',
                            'Joint6']
        self.arm_joints = joints_from_names(self.Robot, arm_joint_names)
        self.sample_fn = get_sample_fn(self.Robot, self.arm_joints)
    
    def motion_planning_disconnect(self):
        disconnect()
        
    def motion_plan(self):
        for _ in range(5):
            print('='*10)

            q1 = list(self.sample_fn())
            # intentionly make the robot needs to cross the collision object
            # let it start from the right side
            q1[0] = 0.
            q1[1] = 0
            q1 = [0,0,0,0,0,0]
            set_joint_positions(self.Robot, self.arm_joints, q1)
            cprint('Sampled start conf: {}'.format(q1), 'cyan')
            wait_for_user()

            # let it ends at the left side
            q2 = list(self.sample_fn())
            q2[0] = 0.5
            q2[1] = 3.0
            q2 = [0.7,0.7,0.7,0.7,0.7,0.7]
            cprint('Sampled end conf: {}'.format(q2), 'cyan')
            # 預設使用rrt connect motion planning
            path = plan_joint_motion(self.Robot, self.arm_joints, q2, obstacles=self.block_obstacles, self_collisions=True,
                custom_limits={self.arm_joints[0]:[0.0, 1.2]})
            if path is None:
                cprint('no plan found', 'red')
                continue
            else:
                wait_for_user('a motion plan is found! Press enter to start simulating!')
            
            cprint('path:{}'.format(path), 'cyan')
        # adjusting this number will adjust the simulation speed
            time_step = 0.03
            for conf in path:
                cprint('path:{}'.format(conf), 'cyan')
                set_joint_positions(self.Robot, self.arm_joints, conf)
                wait_for_duration(time_step)
        # rrt_connect(start_j,start_k,distance_fn = 0.1,sample_fn= ,extend_fn = , collision_fn=True)
    def motion_planning(self, q1, q2, distance = None, obstacles_num = None, collision = True, time_step = 0.03, wait_duration = False):
        # q1 = [0,0,0,0,0,0]
        set_joint_positions(self.Robot, self.arm_joints, q1)
        # wait_for_duration(0.5)
        # q2 = [0.7,0.7,0.7,0.7,0.7,0.7]
        # 預設使用rrt connect motion planning
        path = plan_joint_motion(self.Robot, self.arm_joints, q2, obstacles=self.block_obstacles, self_collisions=collision,
            custom_limits={})
        if path is None:
            # cprint('no plan found', 'red')
            plan_success = False
        # adjusting this number will adjust the simulation speed
        else:
            plan_success = True
            
        time_step = 0.03
        if wait_duration == True and plan_success == True:
            for conf in path:
                cprint('path:{}'.format(conf), 'cyan')
                set_joint_positions(self.Robot, self.arm_joints, conf)
                wait_for_duration(time_step)
        return plan_success, path
    
    def motion_planning_test(self, q1, q2, distance = None, obstacles_num = None, collision = False, time_step = 0.03, wait_duration = True):
        q_test = [np.array([  0.9235,    1.325,    0.599,    -2.26,    1.509,   -1.272]), \
        np.array([ -0.2475,   -1.497,    1.077,    2.115,   0.8647,   0.5595]), \
        np.array([   2.906,  -0.2699,    0.555,   0.5306,   0.8783,  -0.3165]),\
        np.array([ 0.06005,    1.803,   -2.099,   -1.284,   0.7437,  -0.2536]), \
        np.array([    1.24,    1.395,   -1.422, -0.09892,  -0.5599,  -0.1143]), \
        np.array([   1.662,   0.6777,    1.837,   -1.644,    -1.95,    1.452]), \
        np.array([   1.857, -0.08954,   -1.314,    1.235,   0.4439,    2.709]), \
        np.array([ -0.8522,    3.062,    1.905,   -2.086,  -0.1727,   0.9519]), \
        np.array([ -0.8376,    1.661,   -1.618,    3.005,  0.08707,   -1.117]), \
        np.array([ -0.5099,   -1.943,    2.853,    1.774,   0.5494,   -1.742])]
        for i in range(10):
            if i>1:
                set_joint_positions(self.Robot, self.arm_joints, q_test[i-1])
                # wait_for_duration(5)
                # 預設使用rrt connect motion planning
                path = plan_joint_motion(self.Robot, self.arm_joints, q_test[i], obstacles=self.block_obstacles, self_collisions=collision,
                    custom_limits={})
                if path is None:
                    # cprint('no plan found', 'red')
                    plan_success = False
                # adjusting this number will adjust the simulation speed
                else:
                    plan_success = True
                    
                    time_step = 0.03
                    if wait_duration == True:
                        for conf in path:
                            cprint('path:{}'.format(conf), 'cyan')
                            set_joint_positions(self.Robot, self.arm_joints, conf)
                            wait_for_duration(time_step)
                # return plan_success, path
        '''
        robot = modular_robot_6dof(symbolic=False)
        T_tmp = []
        T_tmp.append(SE3(0.25, 0.113, 0.199) * SE3.RPY([np.deg2rad(-173), np.deg2rad(-59), np.deg2rad(-147)]))
        T_tmp.append(SE3(-0.06, -0.09, 0.20) * SE3.RPY([np.deg2rad(-79), np.deg2rad(27), np.deg2rad(-99)]))

        print(robot.ikine_LMS(T=T_tmp[0]))
        print(robot.ikine_LMS(T=T_tmp[1]))
        ik_q1 = robot.ikine_LMS(T=T_tmp[0])
        ik_q2 = robot.ikine_LMS(T=T_tmp[1])
        # q1 = [0,0,0,0,0,0]
        q1 = ik_q1.q
        set_joint_positions(self.Robot, self.arm_joints, q1)
        print(q1)
        # q2 = [0.7,0.7,0.7,0.7,0.7,0.7]
        q2 = ik_q2.q
        print(q2)
        # 預設使用rrt connect motion planning
        path = plan_joint_motion(self.Robot, self.arm_joints, q2, obstacles=self.block_obstacles, self_collisions=collision,
            custom_limits={})
        if path is None:
            # cprint('no plan found', 'red')
            plan_success = False
        # adjusting this number will adjust the simulation speed
        else:
            plan_success = True
            
            time_step = 0.03
            if wait_duration == True:
                for conf in path:
                    cprint('path:{}'.format(conf), 'cyan')
                    set_joint_positions(self.Robot, self.arm_joints, conf)
                    wait_for_duration(time_step)
        return plan_success, path
        '''
    def angle_test(self):
        # robot = modular_robot_6dof(symbolic=False)
        # TODO: 第三軸, 第五軸, 第六軸 方向相反


        q0 = [0,0,0,0,0,0]
        for i in range(0, 360, 15):
            if i >=180:
                q = np.deg2rad([0, 0, 0, 0, 0, 180-i])
            else:
                q = np.deg2rad([0, 0, 0, 0, 0, i])
            print(i)
            set_joint_positions(self.Robot, self.arm_joints, q0)
            path = plan_joint_motion(self.Robot, self.arm_joints, q, self_collisions=True,
            custom_limits={self.arm_joints[0]:[0.0, 1.2]})
            time_step = 0.03
            for conf in path:
                    cprint('path:{}'.format(conf), 'cyan')
                    set_joint_positions(self.Robot, self.arm_joints, conf)
                    wait_for_duration(time_step)
            wait_for_duration(1)

    def test_point(self):
        # TODO: 第三軸, 第五軸, 第六軸 與robotic toolbox方向相反
        q0 = np.deg2rad([0,90,0,23,-90,0]) # test point 1
        q0 = np.deg2rad([0,0,-3,0,0,0]) # test point 2
        # q0 = np.deg2rad([0,76,60,66,-90,0]) # test point 3
        set_joint_positions(self.Robot, self.arm_joints, q0)
        wait_for_duration(100)

    def plane_call(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        planeId = p.loadURDF("plane.urdf")
        
if __name__ == "__main__":

    rospy.init_node("pybullet_test")
    # origin design compare
    # original_design_test = RobotOptEnv()
    # original_design_test.original_design(12,12,44.7,44.7,1.5,50)
    
    motion_bullet= motion_model()
    motion_bullet.reset_robot_urdf(12,12)
    motion_bullet.stl_trimesh_scaling(12, 12)
    # for _ in range(20):
    #     motion_bullet.motion_planning_init(True)
    #     motion_bullet.random_obstacle()
    #     # motion_bullet.motion_plan()
        
    #     q1 = [0,0,0,0,0,0]
    #     q2 = [0.7,0.7,0.7,0.7,0.7,0.7]
    #     plan,path = motion_bullet.motion_planning_test(q1, q2, wait_duration = True)
    #     # sys.stdout = sys.__stdout__
    #     cprint("success:{}".format(plan), 'cyan')
    #     motion_bullet.motion_planning_disconnect()
    # 是否要碰撞可視化
    # 到collision.py 將  diagnosis=False->True
    motion_bullet.motion_planning_init(True)
    motion_bullet.plane_call()
    motion_bullet.test_point()
    '''
    q1 = [0,0,0,0,0,0]
    q2 = [0.7,0.7,0.7,0.7,0.7,0.7]
    motion_bullet.motion_planning_init(True)
    # wait_for_duration(5)
    motion_bullet.random_obstacle()
    motion_bullet.motion_planning_test(q1, q2, wait_duration = True)
    '''
    '''
    motion_bullet.motion_planning_init(True)
    motion_bullet.angle_test()
    '''