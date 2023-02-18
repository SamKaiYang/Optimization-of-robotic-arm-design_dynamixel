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

HERE = os.path.dirname(__file__)
# UR_ROBOT_URDF = os.path.join(HERE,'pybullet_planning_tutorials','examples', 'data', 'universal_robot', 'ur_description', 'urdf', 'ur5.urdf')
SINGLE_ARM = os.path.join(HERE,'urdf', 'single_arm_v12_ori.urdf')
# UR_ROBOT_URDF = os.path.dirname(os.path.realpath(__file__))+"/pybullet_planning_tutorials/examples/data/universal_robot/ur_description/urdf/ur5.urdf"
# physicsClient = pybullet.connect(pybullet.GUI)

# pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

# robot = pybullet.loadURDF

# print(robot.return_configuration())
# physicsClient = p.connect(p.GUI)

# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# planeID = p.loadURDF("plane.urdf")
# Robot = p.loadURDF(os.path.dirname(os.path.realpath(__file__))+"/kuka_experimental/kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.urdf", [0, 0, 0], \
#     useFixedBase=1,flags=p.URDF_USE_SELF_COLLISION)
# p.resetBasePositionAndOrientation(Robot, [0, 0, 0], [0, 0, 0, 1])
# p.setGravity(0 ,0, -9.81)
# p.setRealTimeSimulation(0)
# p.setJointMotorControlArray(Robot, range(7), p.POSITION_CONTROL,\
#     targetPositions=[1.5]*7)

# # for _ in range(300):
# #     p.stepSimulation()
# #     time.sleep(1./10.)
    
# p.resetSimulation()
# planeID = p.loadURDF("plane.urdf")
# Robot = p.loadURDF(os.path.dirname(os.path.realpath(__file__))+"/kuka_experimental/kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.urdf", [0, 0, 0], \
#     useFixedBase=1,flags=p.URDF_USE_SELF_COLLISION)
# p.resetBasePositionAndOrientation(Robot, [0, 0, 0], [0, 0, 0, 1])
# p.setGravity(0 ,0, -9.81)
# p.setRealTimeSimulation(0)
# Orientation = p.getQuaternionFromEuler([3.14,0.,0.])
# targetPositionsJoints = p.calculateInverseKinematics(Robot,7,[0.1,0.1,0.4],\
#     targetOrientation = Orientation)
# p.setJointMotorControlArray(Robot, range(7), p.POSITION_CONTROL,\
#     targetPositions=targetPositionsJoints)

# for _ in range(300):
#     p.stepSimulation()
#     time.sleep(1./10.)

# start_j = [0,1,2,3,5,8,9]
# start_k = [0,30,40,50,60,8,9]

# TODO: gtotkththth

# FIXME: htotot

def random_obstacle():
    pass

def motion_planning():
    pass
def pybullet_init():
    pass

for _ in range(2):
    viewer = True
    connect(use_gui=viewer)
    cprint("Welcome to pybullet! <Ctrl+left mouse> to rotate, <Ctrl+middle mouse> to move the camera, <Ctrl+right mouse> to zoom", 'green')
    cprint('But the space is empty, let\'s load our robot!', 'yellow')
    # wait_for_user is your friend! It will pause the console, but having a separate thread keeping
    # the GUI running so you can rotate and see
    wait_for_user()

    robot_path = SINGLE_ARM
    cprint(robot_path,'green')

    Robot = load_pybullet(robot_path, fixed_base=True)
    arm='right'
    set_camera(yaw=0, pitch=-70, distance=2, target_position=(0, 0, 0))

    cprint('hello RFL! <ctrl+left mouse> to pan', 'green')
    wait_for_user()

    # create a box to be picked up
    # see: https://pybullet-planning.readthedocs.io/en/latest/reference/generated/pybullet_planning.interfaces.env_manager.create_box.html#pybullet_planning.interfaces.env_manager.create_box
    block = create_box(0.01, 0.01, 0.01)
    block_x = 0.3
    block_y = 0.3
    block_z = 0.3
    set_pose(block, Pose(Point(x=block_x, y=block_y, z=block_z), Euler(yaw=np.pi/2)))

    arm_tag = arm[0]
    arm_joint_names = ['Joint1',
                        'Joint2',
                        'Joint3',
                        'Joint4',
                        'Joint5',
                        'Joint6']
    arm_joints = joints_from_names(Robot, arm_joint_names)
    # * if a subset of joints is used, use:
    # arm_joints = joints_from_names(robot, arm_joint_names[1:]) # this will disable the gantry-x joint
    cprint('Used joints: {}'.format(get_joint_names(Robot, arm_joints)), 'yellow')

    # * get a joint configuration sample function:
    # it separately sample each joint value within the feasible range
    sample_fn = get_sample_fn(Robot, arm_joints)

    cprint('Randomly sample robot configuration and set them! (no collision check is performed)', 'blue')
    wait_for_user()
    for i in range(5):
        # randomly sample a joint conf
        sampled_conf = sample_fn()
        set_joint_positions(Robot, arm_joints, sampled_conf)
        cprint('#{} | Conf sampeld: {}'.format(i, sampled_conf), 'green')
        wait_for_user()

    # * now let's plan a trajectory
    # we use y-z-arm 6 joint all together here
    cprint('Randomly sample robot start/end configuration and comptue a motion plan! (no self-collision check is performed)', 'blue')
    print('Disabled collision links needs to be given (can be parsed from a SRDF via compas_fab)')
    for _ in range(5):
        print('='*10)

        q1 = list(sample_fn())
        # intentionly make the robot needs to cross the collision object
        # let it start from the right side
        q1[0] = 0.
        q1[1] = 0
        q1 = [0,0,0,0,0,0]
        set_joint_positions(Robot, arm_joints, q1)
        cprint('Sampled start conf: {}'.format(q1), 'cyan')
        wait_for_user()

        # let it ends at the left side
        q2 = list(sample_fn())
        q2[0] = 0.5
        q2[1] = 3.0
        q2 = [0.3,0.3,0.3,0.3,0.3,0.3]
        cprint('Sampled end conf: {}'.format(q2), 'cyan')
        # 預設使用rrt connect motion planning
        path = plan_joint_motion(Robot, arm_joints, q2, obstacles=[block], self_collisions=True,
            custom_limits={arm_joints[0]:[0.0, 1.2]})
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
            set_joint_positions(Robot, arm_joints, conf)
            wait_for_duration(time_step)
    # rrt_connect(start_j,start_k,distance_fn = 0.1,sample_fn= ,extend_fn = , collision_fn=True)

    disconnect()
# p.getNumJoints(Robot)

# position, orientation = p.getBasePositionAndOrientation(Robot)

# time.sleep(1000)