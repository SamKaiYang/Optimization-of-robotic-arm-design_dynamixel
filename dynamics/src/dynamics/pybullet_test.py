#!/usr/bin/env python3
# coding: utf-8
import rospy
import pybullet as p
import time
import pybullet_data
import os
from termcolor import cprint
import numpy as np
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
from pybullet_planning import get_collision_fn, get_floating_body_collision_fn, expand_links, create_box
from pybullet_planning import pairwise_collision, pairwise_collision_info, draw_collision_diagnosis, body_collision_info
from pybullet_planning import rrt_connect

import trimesh
from stl_conv_6dof_urdf_dynamixel import stl_conv_urdf

HERE = os.path.dirname(__file__)
SINGLE_ARM = os.path.join(HERE,'urdf', 'single_arm_v12_motion.urdf')

class motion_model(object):
    def __init__(self):
        self.length_1 = 0.1
        self.length_2 = 0.1
        self.length_3 = 0.1
    def random_obstacle(self):
        # create a box to be picked up
        # see: https://pybullet-planning.readthedocs.io/en/latest/reference/generated/pybullet_planning.interfaces.env_manager.create_box.html#pybullet_planning.interfaces.env_manager.create_box
        self.block = create_box(0.01, 0.01, 0.01)
        block_x = 0.3
        block_y = 0.3
        block_z = 0.3
        set_pose(self.block, Pose(Point(x=block_x, y=block_y, z=block_z), Euler(yaw=np.pi/2)))

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
        viewer = True
        connect(use_gui=viewer)
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
            q2 = [0.3,0.3,0.3,0.3,0.3,0.3]
            cprint('Sampled end conf: {}'.format(q2), 'cyan')
            # 預設使用rrt connect motion planning
            path = plan_joint_motion(self.Robot, self.arm_joints, q2, obstacles=[self.block], self_collisions=True,
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
if __name__ == "__main__":

    rospy.init_node("pybullet_test")
    motion_bullet= motion_model()
    motion_bullet.reset_robot_urdf(60,60)
    motion_bullet.stl_trimesh_scaling(60, 60)
    motion_bullet.motion_planning_init(True)
    motion_bullet.random_obstacle()
    motion_bullet.motion_plan()