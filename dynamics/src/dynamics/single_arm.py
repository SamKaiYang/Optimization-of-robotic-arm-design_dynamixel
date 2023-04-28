#!/usr/bin/env python3
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
import matplotlib.pyplot as plt
from spatialmath import SE3
from urdf_parser_py.urdf import URDF
import os
import openpyxl
import pandas as pd
import math as pi
from openpyxl import load_workbook
class single_arm(DHRobot):
    """
    Class that models a TECO TECOARM1 manipulator

    :param symbolic: use symbolic constants
    :type symbolic: bool

    describes its kinematic and dynamic characteristics using standard DH
    conventions.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.DH.TECOARM1()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration
    - qr, arm horizontal along x-axis

    .. note::
        - SI units are used.

    :References:

    .. codeauthor:: Yi He Yang
    """  # noqa

    def __init__(self, symbolic=False):
        # path = os.path.dirname(os.path.realpath(__file__))+"/single_arm.urdf"
        # print(path)
        # robot = URDF.from_xml_file(path)
        if symbolic:
            import spatialmath.base.symbolic as sym
            zero = sym.zero()
            pi = sym.pi()
        else:
            from math import pi
            zero = 0.0

        deg = pi / 180
        inch = 0.0254
        # robot length values (metres)
        a = [0, 0, -0.03, 0.03, 0, 0, 0] #m # single arm
        # a = [0, -robot.joints[3].origin.xyz[1], -robot.joints[4].origin.xyz[1], 0, 0, 0]
        # a = [0, -j3.y, -j4.y, 0, 0, 0]
        d = [0.2463, 0, 0.29, 0, 0.27, 0, 0.24] #m # single arm 
        # d = [robot.joints[1].origin.xyz[2],
        #     0,
        #     0,
        #     robot.joints[2].origin.xyz[1]-robot.joints[4].origin.xyz[2], 
        #     robot.joints[5].origin.xyz[2],
        #     robot.joints[6].origin.xyz[2]
        # ]
        # d = [j1.z, 0, 0, j2.y-j4.z , j5.z, j6.z] #m
        alpha = [-pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, zero]
        theta = [zero, zero, zero, zero, zero, zero, zero]

        
        # mass data, no inertia available
        mass = [0.386131223153925, 0.479430893999342, 0.568306813792926, 0.457613402476782, 0.448277275798768, 0.205818407044008, 0.3927]
        # mass = [robot.links[2].inertial.mass, 
        #         robot.links[3].inertial.mass,
        #         robot.links[4].inertial.mass,
        #         robot.links[5].inertial.mass,
        #         robot.links[6].inertial.mass,
        #         robot.links[7].inertial.mass
        #     ]
        
        G= [-80,-80,-80,-50,-50,-50, -50]   # gear ratio

        center_of_mass = [
                # [0.000156250860732315 0.000202963757501797 -0.0546720420122521],
                [-3.64540734812575E-06, 0.00119778550610652, -0.000421399675048756],
                [0.000683310419598862, -8.00535195483731E-05, 0.117541995924546],
                [-0.0220564838621471, 0.101455973505417, -8.62457453022769E-05],
                [0.0297389855299131, 0.0361651889050357, 0.108377256276242],
                [-0.000172095429412281, 0.102690028206534, -6.66794215013755E-05],
                [1.14707347786958E-08, 0.0359051244401507, 0.076720238383497],
                [-4.4856E-13, 0, 0.025]
            ]

        # center_of_mass = [
        #         robot.links[2].inertial.origin.xyz,
        #         robot.links[3].inertial.origin.xyz,
        #         robot.links[4].inertial.origin.xyz,
        #         robot.links[5].inertial.origin.xyz,
        #         robot.links[6].inertial.origin.xyz,
        #         robot.links[7].inertial.origin.xyz
        #     ]

        # ixx,iyy,izz,ixy,iyz,ixz
        inertia = [
                [0.1 ,0.1, 0.1, 0.0, 0.0, 0.0 ],
                [0.1 ,0.1, 0.1, 0.0, 0.0, 0.0 ],
                [0.1 ,0.1, 0.1, 0.0, 0.0, 0.0 ],
                [0.1 ,0.1, 0.1, 0.0, 0.0, 0.0 ],
                [0.1 ,0.1, 0.1, 0.0, 0.0, 0.0 ],
                [0.1 ,0.1, 0.1, 0.0, 0.0, 0.0 ],
                [0.1 ,0.1, 0.1, 0.0, 0.0, 0.0 ]
            ]
        links = []

        for j in range(7):
            link = RevoluteDH(
                d=d[j],
                alpha=alpha[j],
                # theta=theta[j],
                a=a[j],
                m=mass[j],
                r=center_of_mass[j],
                I=inertia[j],
                G=G[j]
            )
            links.append(link)
    
        super().__init__(
            links,
            name="robot",
            manufacturer="Robotics",
            keywords=('dynamics', 'symbolic'),
            symbolic=symbolic
        )
    
        # zero angles
        # theta = [zero, pi, zero, zero, zero, zero, zero]
        self.addconfiguration("qz", np.array([90, -90, 0, 0, 0, 0, 90])*deg)
        # horizontal along the x-axis
        # self.addconfiguration("qr", np.r_[0, 0, 0, 0, 0, 0, 0]*deg)

        # nominal table top picking pose
        self.addconfiguration("qn", np.array([90, -90, 0, 0, 0, 0, 90])*deg)
if __name__ == '__main__':    # pragma nocover

    robot = single_arm(symbolic=False)
    print(robot)
    print(robot.dynamics())
    # TODO: fixed DH table
    from math import pi
    deg = pi / 180
    qz = np.array([0, 0, 0, 0, 0, 0, 0])*deg
    # robot.plot(qz, dt = 10)
    
    # FIXME: 如有負載要記得掛負載
    payload = 0
    payload_position =  [0, 0, 0.0]
    robot.payload(payload, payload_position)  # set payload
    # TODO: 讀取excel，帶入traj 計算扭矩 
    # TODO: 將兩組資訊繪圖出來
    # 讀取excel檔案
    df = pd.read_excel("joint_states_cmd.xlsx",engine='openpyxl',)

    # 讀取兩筆資料
    df1 = pd.read_excel("joint_states_ori.xlsx", engine='openpyxl')
    df2 = pd.read_excel("joint_states_cmd.xlsx", engine='openpyxl')

    # 計算誤差
    error_1 = np.sqrt(np.mean((df1['P_joint1'] - df2['P_joint1']) ** 2))
    error_2 = np.sqrt(np.mean((df1['P_joint2'] - df2['P_joint2']) ** 2))
    error_3 = np.sqrt(np.mean((df1['P_joint3'] - df2['P_joint3']) ** 2))
    error_4 = np.sqrt(np.mean((df1['P_joint4'] - df2['P_joint4']) ** 2))
    error_5 = np.sqrt(np.mean((df1['P_joint5'] - df2['P_joint5']) ** 2))
    error_6 = np.sqrt(np.mean((df1['P_joint6'] - df2['P_joint6']) ** 2))
    error_7 = np.sqrt(np.mean((df1['P_joint7'] - df2['P_joint7']) ** 2))
    error = (error_1 + error_2 + error_3 + error_4 + error_5 + error_6 + error_7)/7
    print("位置均方根誤差（RMSE）：", error)

    # 求取最大差異值
    # max_diff = np.max(np.abs((df1['P_joint1'] - df2['P_joint1'])))
    diff = df1['P_joint1'] - df2['P_joint1']
    max_diff_percent = np.max(np.abs(diff))/np.max(np.abs(df2['P_joint1']))*100
    # 求取平均差異值
    # mean_diff = np.mean(np.abs((df1['P_joint1'] - df2['P_joint1'])))
    mean_diff_percent = np.mean(np.abs(diff))/np.mean(np.abs(df2['P_joint1']))*100
    print('max_diff_percent:', max_diff_percent)
    print('mean_diff_percent:', mean_diff_percent)
    # # 計算速度和加速度
    # df['v_joint1'] = df['P_joint1'].diff() / df['time'].diff()
    # df['a_joint1'] = df['v_joint1'].diff() / df['time'].diff()
    
    # # 去除第一行(因為第一行無法計算差分)
    # df = df.drop([0])

    # # 印出計算後的結果
    # # print(df)

    # # 繪製速度和加速度的折線圖
    # plt.plot(df['time'], df['v_joint1'], label='velocity')
    # plt.plot(df['time'], df['a_joint1'], label='acceleration')
    # # col1 = np.array(df.iloc[:, 1:8])
    # pos_data = np.array([])
    # pos_data = np.append(pos_data, np.array(df['P_joint1']))
    # vel_data = np.array(df['v_joint1'])
    # acc_data = np.array(df['a_joint1'])
    # torque = []
    # # TODO: 只有一軸資訊 還無法算動力學
    time = df['time']
    pos_data = df[['P_joint1', 'P_joint2', 'P_joint3', 'P_joint4', 'P_joint5', 'P_joint6', 'P_joint7']].to_numpy()
    fig, (ax1, ax2, ax3, ax4, ax5, ax6, ax7) = plt.subplots(7, 1)
    ax1.plot(time, pos_data[:, 0], label='P_joint1', color='blue', linewidth=2, linestyle='solid')
    ax2.plot(time, pos_data[:, 1], label='P_joint2', color='red', linewidth=2, linestyle='dashed')
    ax3.plot(time, pos_data[:, 2], label='P_joint3', color='green', linewidth=2, linestyle='dotted')
    ax4.plot(time, pos_data[:, 3], label='P_joint4', color='orange', linewidth=2, linestyle='dashdot')
    ax5.plot(time, pos_data[:, 4], label='P_joint5', color='purple', linewidth=2, linestyle='solid')
    ax6.plot(time, pos_data[:, 5], label='P_joint6', color='brown', linewidth=2, linestyle='dashed')
    ax7.plot(time, pos_data[:, 6], label='P_joint7', color='pink', linewidth=2, linestyle='dotted')
    plt.show()
    # 初始化速度和加速度數據的陣列
    # vel_tuple = (3247,7)
    # acc_tuple = (3246,7)
    vel_data = np.zeros(pos_data.shape)
    acc_data = np.zeros(pos_data.shape)

    # 對每個軸的位置數據進行求解
    for i in range(pos_data.shape[1]):
        vel_data[:, i] = np.gradient(pos_data[:, i], time)
        acc_data[:, i] = np.gradient(vel_data[:, i], time)
        # vel_data[:, i] = np.diff(pos_data[:, i]) / np.diff(time)
        # acc_data[:, i] = np.diff(vel_data[:, i]) / np.diff(time[:-1])
    # 繪製速度和加速度的圖表 (cmd)
    fig, (ax1, ax2, ax3, ax4, ax5, ax6, ax7) = plt.subplots(7, 1)
    ax1.plot(time, vel_data[:, 0], label='P_joint1', color='blue', linewidth=2, linestyle='solid')
    ax2.plot(time, vel_data[:, 1], label='P_joint2', color='red', linewidth=2, linestyle='dashed')
    ax3.plot(time, vel_data[:, 2], label='P_joint3', color='green', linewidth=2, linestyle='dotted')
    ax4.plot(time, vel_data[:, 3], label='P_joint4', color='orange', linewidth=2, linestyle='dashdot')
    ax5.plot(time, vel_data[:, 4], label='P_joint5', color='purple', linewidth=2, linestyle='solid')
    ax6.plot(time, vel_data[:, 5], label='P_joint6', color='brown', linewidth=2, linestyle='dashed')
    ax7.plot(time, vel_data[:, 6], label='P_joint7', color='pink', linewidth=2, linestyle='dotted')
    plt.title('Joint vel cmd')
    plt.show()
    # obser
    fig, (ax1, ax2, ax3, ax4, ax5, ax6, ax7) = plt.subplots(7, 1)
    ax1.plot(df1['time'], df1['V_joint1'], label='P_joint1', color='blue', linewidth=2, linestyle='solid')
    ax2.plot(df1['time'], df1['V_joint1'], label='P_joint2', color='red', linewidth=2, linestyle='dashed')
    ax3.plot(df1['time'], df1['V_joint1'], label='P_joint3', color='green', linewidth=2, linestyle='dotted')
    ax4.plot(df1['time'], df1['V_joint1'], label='P_joint4', color='orange', linewidth=2, linestyle='dashdot')
    ax5.plot(df1['time'], df1['V_joint1'], label='P_joint5', color='purple', linewidth=2, linestyle='solid')
    ax6.plot(df1['time'], df1['V_joint1'], label='P_joint6', color='brown', linewidth=2, linestyle='dashed')
    ax7.plot(df1['time'], df1['V_joint1'], label='P_joint7', color='pink', linewidth=2, linestyle='dotted')
    plt.title('Joint vel ori')
    plt.show()
    # 計算速度誤差
    error_1 = np.sqrt(np.mean((df1['V_joint1'] - vel_data[:, 0]) ** 2))
    error_2 = np.sqrt(np.mean((df1['V_joint2'] - vel_data[:, 1]) ** 2))
    error_3 = np.sqrt(np.mean((df1['V_joint3'] - vel_data[:, 2]) ** 2))
    error_4 = np.sqrt(np.mean((df1['V_joint4'] - vel_data[:, 3]) ** 2))
    error_5 = np.sqrt(np.mean((df1['V_joint5'] - vel_data[:, 4]) ** 2))
    error_6 = np.sqrt(np.mean((df1['V_joint6'] - vel_data[:, 5]) ** 2))
    error_7 = np.sqrt(np.mean((df1['V_joint7'] - vel_data[:, 6]) ** 2))
    error = (error_1 + error_2 + error_3 + error_4 + error_5 + error_6 + error_7)/7
    print("速度均方根誤差（RMSE）：", error)

    diff = df1['V_joint1'] - vel_data[:, 0]
    max_diff_percent = np.max(np.abs(diff))/np.max(np.abs( vel_data[:, 0]))*100
    # 求取平均差異值
    # mean_diff = np.mean(np.abs((df1['P_joint1'] - df2['P_joint1'])))
    mean_diff_percent = np.mean(np.abs(diff))/np.mean(np.abs( vel_data[:, 0]))*100
    print('max_diff_percent:', max_diff_percent)
    print('mean_diff_percent:', mean_diff_percent)
    # plt.plot(time, vel_data)
    # plt.title('Joint Velocities')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Velocity (rad/s)')
    # plt.legend(['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'])

    fig, (ax1, ax2, ax3, ax4, ax5, ax6, ax7) = plt.subplots(7, 1)
    ax1.plot(time, acc_data[:, 0], label='P_joint1', color='blue', linewidth=2, linestyle='solid')
    ax2.plot(time, acc_data[:, 1], label='P_joint2', color='red', linewidth=2, linestyle='dashed')
    ax3.plot(time, acc_data[:, 2], label='P_joint3', color='green', linewidth=2, linestyle='dotted')
    ax4.plot(time, acc_data[:, 3], label='P_joint4', color='orange', linewidth=2, linestyle='dashdot')
    ax5.plot(time, acc_data[:, 4], label='P_joint5', color='purple', linewidth=2, linestyle='solid')
    ax6.plot(time, acc_data[:, 5], label='P_joint6', color='brown', linewidth=2, linestyle='dashed')
    ax7.plot(time, acc_data[:, 6], label='P_joint7', color='pink', linewidth=2, linestyle='dotted')
    plt.show()
    # plt.figure()
    # plt.plot(time, acc_data)
    # plt.title('Joint Accelerations')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Acceleration (rad/s^2)')
    # plt.legend(['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'])

    # plt.show()
# ----------------------------------------------------------------
    # CMD

    plt.plot(time, pos_data[:, 0], label='P_joint1', color='blue', linewidth=2, linestyle='solid')
    plt.plot(time, pos_data[:, 1], label='P_joint2', color='red', linewidth=2, linestyle='dashed')
    plt.plot(time, pos_data[:, 2], label='P_joint3', color='green', linewidth=2, linestyle='dotted')
    plt.plot(time, pos_data[:, 3], label='P_joint4', color='orange', linewidth=2, linestyle='dashdot')
    plt.plot(time, pos_data[:, 4], label='P_joint5', color='purple', linewidth=2, linestyle='solid')
    plt.plot(time, pos_data[:, 5], label='P_joint6', color='brown', linewidth=2, linestyle='dashed')
    plt.plot(time, pos_data[:, 6], label='P_joint7', color='pink', linewidth=2, linestyle='dotted')
    plt.title('Simulation:Joint Position')
    plt.show()

    plt.plot(time, vel_data[:, 0], label='P_joint1', color='blue', linewidth=2, linestyle='solid')
    plt.plot(time, vel_data[:, 1], label='P_joint2', color='red', linewidth=2, linestyle='dashed')
    plt.plot(time, vel_data[:, 2], label='P_joint3', color='green', linewidth=2, linestyle='dotted')
    plt.plot(time, vel_data[:, 3], label='P_joint4', color='orange', linewidth=2, linestyle='dashdot')
    plt.plot(time, vel_data[:, 4], label='P_joint5', color='purple', linewidth=2, linestyle='solid')
    plt.plot(time, vel_data[:, 5], label='P_joint6', color='brown', linewidth=2, linestyle='dashed')
    plt.plot(time, vel_data[:, 6], label='P_joint7', color='pink', linewidth=2, linestyle='dotted')
    plt.title('Simulation:Joint Velocity')
    plt.show()
    
    torque = robot.rne(pos_data,vel_data,acc_data, gravity=[0,0,9.81])
    # torque = np.array(torque)
    plt.plot(time, -20/6*torque[:,0], label='E_joint1', color='blue', linewidth=2, linestyle='solid')
    plt.plot(time, -20/6*torque[:,1], label='E_joint2', color='red', linewidth=2, linestyle='dashed')
    plt.plot(time, -20/6*torque[:,2], label='E_joint3', color='green', linewidth=2, linestyle='dotted')
    plt.plot(time, -20/6*torque[:,3], label='E_joint4', color='orange', linewidth=2, linestyle='dashdot')
    plt.plot(time, -20/6*torque[:,4], label='E_joint5', color='purple', linewidth=2, linestyle='solid')
    plt.plot(time, -20/6*torque[:,5], label='E_joint6', color='brown', linewidth=2, linestyle='dashed')
    plt.plot(time, -20/6*torque[:,6], label='E_joint7', color='pink', linewidth=2, linestyle='dotted')
    plt.title('Simulation:Joint Torque')
    plt.show()
# ----------------------------------------------------------------
    # REAL
    # obser

    plt.plot(df1['time'], df1['P_joint1'], label='P_joint1', color='blue', linewidth=2, linestyle='solid')
    plt.plot(df1['time'], df1['P_joint2'], label='P_joint2', color='red', linewidth=2, linestyle='dashed')
    plt.plot(df1['time'], df1['P_joint3'], label='P_joint3', color='green', linewidth=2, linestyle='dotted')
    plt.plot(df1['time'], df1['P_joint4'], label='P_joint4', color='orange', linewidth=2, linestyle='dashdot')
    plt.plot(df1['time'], df1['P_joint5'], label='P_joint5', color='purple', linewidth=2, linestyle='solid')
    plt.plot(df1['time'], df1['P_joint6'], label='P_joint6', color='brown', linewidth=2, linestyle='dashed')
    plt.plot(df1['time'], df1['P_joint7'], label='P_joint7', color='pink', linewidth=2, linestyle='dotted')
    plt.title('Real:Joint Position')
    plt.show()

    plt.plot(df1['time'], df1['V_joint1'], label='V_joint1', color='blue', linewidth=2, linestyle='solid')
    plt.plot(df1['time'], df1['V_joint2'], label='V_joint2', color='red', linewidth=2, linestyle='dashed')
    plt.plot(df1['time'], df1['V_joint3'], label='V_joint3', color='green', linewidth=2, linestyle='dotted')
    plt.plot(df1['time'], df1['V_joint4'], label='V_joint4', color='orange', linewidth=2, linestyle='dashdot')
    plt.plot(df1['time'], df1['V_joint5'], label='V_joint5', color='purple', linewidth=2, linestyle='solid')
    plt.plot(df1['time'], df1['V_joint6'], label='V_joint6', color='brown', linewidth=2, linestyle='dashed')
    plt.plot(df1['time'], df1['V_joint7'], label='V_joint7', color='pink', linewidth=2, linestyle='dotted')
    plt.title('Real:Joint Velocity')
    plt.show()

    plt.plot(time, df1['E_joint1'], label='E_joint1', color='blue', linewidth=2, linestyle='solid')
    plt.plot(time, df1['E_joint2'], label='E_joint2', color='red', linewidth=2, linestyle='dashed')
    plt.plot(time, df1['E_joint3'], label='E_joint3', color='green', linewidth=2, linestyle='dotted')
    plt.plot(time, df1['E_joint4'], label='E_joint4', color='orange', linewidth=2, linestyle='dashdot')
    plt.plot(time, df1['E_joint5'], label='E_joint5', color='purple', linewidth=2, linestyle='solid')
    plt.plot(time, df1['E_joint6'], label='E_joint6', color='brown', linewidth=2, linestyle='dashed')
    plt.plot(time, df1['E_joint7'], label='E_joint7', color='pink', linewidth=2, linestyle='dotted')
    plt.title('Real:Joint Torque')
    plt.show()

    # 扭矩誤差
    error_1 = np.sqrt(np.mean((df1['E_joint1'] - torque[:, 0]) ** 2))
    error_2 = np.sqrt(np.mean((df1['E_joint2'] - torque[:, 1]) ** 2))
    error_3 = np.sqrt(np.mean((df1['E_joint3'] - torque[:, 2]) ** 2))
    error_4 = np.sqrt(np.mean((df1['E_joint4'] - torque[:, 3]) ** 2))
    error_5 = np.sqrt(np.mean((df1['E_joint5'] - torque[:, 4]) ** 2))
    error_6 = np.sqrt(np.mean((df1['E_joint6'] - torque[:, 5]) ** 2))
    error_7 = np.sqrt(np.mean((df1['E_joint7'] - torque[:, 6]) ** 2))
    error = (error_1 + error_2 + error_3 + error_4 + error_5 + error_6 + error_7)/7
    print("扭矩均方根誤差（RMSE）：", error)

    diff = df1['E_joint2'] - (-20/6*torque[:, 1])
    max_diff_percent = np.max(np.abs(diff))/np.max(np.abs(torque[:, 1]))*100
    # 求取平均差異值
    mean_diff_percent = np.mean(np.abs(diff))/np.mean(np.abs(torque[:, 1]))*100
    print('max_diff_percent:', max_diff_percent)
    print('mean_diff_percent:', mean_diff_percent)
        # 計算誤差的平均值和標準差
    # mean_error = merged['error'].mean()
    # std_error = merged['error'].std()

    # print(f"Mean error: {mean_error}")
    # print(f"Standard deviation of error: {std_error}")