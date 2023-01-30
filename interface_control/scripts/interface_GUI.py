#!/usr/bin/env python3
# -*-coding:utf-8 -*-
import rospy
import numpy as np

# from dynamics.dynamics_function_teco import Dynamics_teco

from PySide2 import QtWidgets, QtGui
from PySide2.QtCore import *
from PySide2.QtWidgets import *
from PySide2.QtGui import *
from PySide2extn.RoundProgressBar import roundProgressBar #IMPORT THE EXTENSION LIBRARY

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from Ui_main import Ui_MainWindow
from interface_control.msg import (arm_structure, cal_cmd, dyna_data, dyna_space_data,
                                specified_parameter_design, cal_process, cal_result, 
                                communicate_matlab, optimal_design, tested_model_name)
from std_msgs.msg import String
import sys
import importlib
importlib.reload(sys)

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
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False

class MyThread(QThread):
    callback = Signal(int, int)#自定義訊號, Qt的文件中有說明, 必需為類別變數
    def __init__(self, label, delay, parent=None):
        super(MyThread, self).__init__(parent)
        self.runFlag = True
        self.label=label
        self.delay=delay
        
    def __del__(self):
        self.runFlag = False
        self.wait()

    def run(self):
        index=0
        while self.runFlag:
            self.callback.emit(index, self.label)
            index+=1
            self.msleep(self.delay)


    def setData(self, index, value, role):
        if role == Qt.EditRole:
            try:
                value = int(value)
            except ValueError:
                return False
            self._data[index.row(), index.column()] = value
            return True
        return False

    def flags(self, index):
        return Qt.ItemIsSelectable | Qt.ItemIsEnabled | Qt.ItemIsEditable

class MyFigureCanvas(FigureCanvas):
    def __init__(self, parent=None, width=10, height=5, xlim=(0, 2500), ylim=(-2, 2), dpi=100):
        # 创建一个Figure
        fig = plt.Figure(figsize=(width, height), dpi=dpi, tight_layout=True) # tight_layout: 用于去除画图时两边的空白

        FigureCanvas.__init__(self, fig) # 初始化父类
        self.setParent(parent)

        self.axes = fig.add_subplot(111) # 添加子图
        self.axes.spines['top'].set_visible(False) # 去掉绘图时上面的横线
        self.axes.spines['right'].set_visible(False) # 去掉绘图时右面的横线
        self.axes.set_xlim(xlim)
        self.axes.set_ylim(ylim)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # 初始化 gv_visual_data 的显示
        # self.gv_visual_data_content = MyFigureCanvas(width=self.ui.graphicsView.width() / 101,
        #                     height=self.ui.graphicsView.height() / 101,
        #                     xlim=(0, 2*np.pi),
        #                     ylim=(-1, 1)) # 实例化一个FigureCanvas
        # publish cmd to dynamics calculate : use function select 
        self.pub_cmd = rospy.Publisher("/cal_command",cal_cmd, queue_size=10)
        # publish arm dynamics data : float32 payload, joint_angle, payload_position, vel, acc
        self.pub_dyna_data = rospy.Publisher("/dynamics_data",dyna_data, queue_size=10)
        # publish arm dynamics use space scan data 
        self.pub_dyna_space = rospy.Publisher("/dynamics_space_data",dyna_space_data, queue_size=10)
        # publish arm design data : arm length, payload, dof
        self.pub_specified_parameter_design = rospy.Publisher("/specified_parameter_design",specified_parameter_design, queue_size=10)
        # publish command for matlab program 
        self.pub_communicate_matlab = rospy.Publisher("/communicate_matlab",String, queue_size=10)
        # publish data for optimal design 
        self.pub_optimal_design = rospy.Publisher("/optimal_design",optimal_design, queue_size=10)
        # suscribe dynamics calculate process 
        self.sub_dyna_space_progress = rospy.Subscriber("/dyna_space_progress",cal_process, self.dyna_cal_process_callback)
        # publish select dof and structure 
        self.pub_arm_structure = rospy.Publisher("/arn_structure", arm_structure, queue_size=10)
        # publish tested model name
        self.tested_model_name = rospy.Publisher("/tested_model_name", tested_model_name, queue_size=10)

        self.cal_cmd = cal_cmd()
        self.dyna_data = dyna_data()
        self.dyna_space_data = dyna_space_data()
        self.specified_parameter_design = specified_parameter_design()
        self.communicate_matlab = String()
        self.optimal_design = optimal_design()
        self.arm_structure = arm_structure()
        self.tested_model = tested_model_name()
        
        self.rpb = self.ui.widget
        self.rpb.rpb_setValue(0)

        self.payload = 0.0
        self.payload_position = [0.0,0.0,0.0]
        self.payload_space = 0.0
        self.payload_position_space = [0.0,0.0,0.0]
        self.joint_velocity = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_acceleration = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_angle = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]

        # arm design parameters
        self.axis_2 = 20
        self.axis_3 = 20
        self.dof = 6
        self.dyna_space_progress = 0
        
        # op design
        self.dof_select = 6
        self.algorithm_name = "test"

        self.ui.btn_dynamics.clicked.connect(self.dyna_buttonClicked)
        self.ui.btn_dyn_space.clicked.connect(self.dyna_space_buttonClicked)
        self.ui.btn_dyn_set.clicked.connect(self.dyna_set_buttonClicked)
        self.ui.btn_dyn_space_set.clicked.connect(self.dyna_space_set_buttonClicked)
        self.ui.btn_arm_plot.clicked.connect(self.arm_plot_buttonClicked)
        self.ui.btn_dyn_axis_set.clicked.connect(self.axis_set_buttonClicked)
        self.ui.btn_arm_plot_close.clicked.connect(self.arm_plot_close_buttonClicked)
        self.ui.btn_dynamics_design.clicked.connect(self.dynamics_design_buttonClicked)
        self.ui.btn_dyn_torque_limit.clicked.connect(self.dyn_torque_limit_buttonClicked)
        self.ui.btn_dynamics_new_traj.clicked.connect(self.dynamics_new_traj_buttonClicked)
        self.ui.btn_dynamics_real_torq.clicked.connect(self.dynamics_real_torq_buttonClicked)
        self.ui.btn_dynamics_save_para.clicked.connect(self.dynamics_save_para_buttonClicked)
        self.ui.btn_random_robot_structure.clicked.connect(self.random_robot_structure_buttonClicked)
        self.ui.btn_random_robot_motor.clicked.connect(self.random_robot_motor_buttonClicked)
        self.ui.btn_optimization_analysis.clicked.connect(self.optimization_analysis_buttonClicked)
        self.ui.btn_optimization_set.clicked.connect(self.optimization_set_buttonClicked)
        self.ui.btn_cjm_select.clicked.connect(self.cjm_select_buttonClicked)
        self.ui.btn_traj_torque_plot.clicked.connect(self.traj_torque_plot_buttonClicked)
        self.ui.btn_structure_select.clicked.connect(self.structure_set_buttonClicked)
        self.ui.btn_optimization_test.clicked.connect(self.btn_optimization_test_buttonClicked)
        # # Vel. HorizontalSlider
        # self.ui.horizontalSlider_vel.valueChanged.connect(self.VelSliderValue)
        # # Acc. HorizontalSlider
        # self.ui.horizontalSlider_acc.valueChanged.connect(self.AccSliderValue)

        # ComboBox axis 2 length
        choices = ['0.15','0.2','0.25','0.3','0.35','0.4','0.45','0.5']
        self.ui.comboBox_2_length.addItems(choices)
        self.ui.comboBox_2_length.currentIndexChanged.connect(self.display_2_length)
        self.ui.comboBox_2_length.setCurrentIndex(1)
        self.display_2_length()
        
        # ComboBox axis 3 length
        choices = ['0.15','0.2','0.25','0.3','0.35','0.4','0.45','0.5']
        self.ui.comboBox_3_length.addItems(choices)
        self.ui.comboBox_3_length.currentIndexChanged.connect(self.display_3_length)
        self.ui.comboBox_3_length.setCurrentIndex(1)
        self.display_3_length()

        # ComboBox dof
        choices = ['1','2', '3', '4', '5','6','7']
        self.ui.comboBox_dof.addItems(choices)
        self.ui.comboBox_dof.currentIndexChanged.connect(self.display_dof)
        self.ui.comboBox_dof.setCurrentIndex(5)
        self.display_dof()
        
        # ComboBox structure dof select 
        choices = ['1','2', '3', '4', '5','6','7']
        self.ui.comboBox_dof_select.addItems(choices)
        self.ui.comboBox_dof_select.currentIndexChanged.connect(self.display_dof_select)
        self.ui.comboBox_dof_select.setCurrentIndex(5)
        self.display_dof_select()
        # # ComboBox algorithm name
        # choices = ['random','TECO','UR','TM','Single Arm','SCARA','3DOF']
        # self.ui.comboBox_algorithm_select.addItems(choices)
        # self.ui.comboBox_algorithm_select.currentIndexChanged.connect(self.display_structure_select)
        # self.ui.comboBox_algorithm_select.setCurrentIndex(0)
        # self.display_structure_select()

        # ComboBox algorithm name
        choices = ['DQN','DDQN','C51']
        self.ui.comboBox_algorithm_select.addItems(choices)
        self.ui.comboBox_algorithm_select.currentIndexChanged.connect(self.display_structure_select)
        self.ui.comboBox_algorithm_select.setCurrentIndex(0)
        self.display_structure_select()
        # self.progressBar_dynamics_space = QtWidgets.QProgressBar(self.ui.graphicsView_2)
    def dyna_cal_process_callback(self,data):
        # dyna_cal_process_callback
        self.dyna_space_progress = data.dyna_space_progress
        # rospy.loginfo("I heard command is %s", data.dyna_space_progress)
        self.rpb.rpb_setValue(self.dyna_space_progress) #CHANGING THE VALUE OF THE PROGRESS BAR


    def display_2_length(self):
        if self.ui.comboBox_2_length.currentText() == "0.15":
            self.axis_2 = 0.15*100
            self.ui.comboBox_2_length.setCurrentIndex(0)
        elif self.ui.comboBox_2_length.currentText() == "0.2":
            self.axis_2 = 0.2*100
            self.ui.comboBox_2_length.setCurrentIndex(1)
        elif self.ui.comboBox_2_length.currentText() == "0.25":
            self.axis_2 = 0.25*100
            self.ui.comboBox_2_length.setCurrentIndex(2)
        elif self.ui.comboBox_2_length.currentText() == "0.3":
            self.axis_2 = 0.3*100
            self.ui.comboBox_2_length.setCurrentIndex(3)
        elif self.ui.comboBox_2_length.currentText() == "0.35":
            self.axis_2 = 0.35*100
            self.ui.comboBox_2_length.setCurrentIndex(4)
        elif self.ui.comboBox_2_length.currentText() == "0.4":
            self.axis_2 = 0.4*100
            self.ui.comboBox_2_length.setCurrentIndex(5)
        elif self.ui.comboBox_2_length.currentText() == "0.45":
            self.axis_2 = 0.45*100
            self.ui.comboBox_2_length.setCurrentIndex(6)
        elif self.ui.comboBox_2_length.currentText() == "0.5":
            self.axis_2 = 0.5*100
            self.ui.comboBox_2_length.setCurrentIndex(7)

    def display_3_length(self):
        if self.ui.comboBox_3_length.currentText() == "0.15":
            self.axis_3 = 0.15*100
            self.ui.comboBox_3_length.setCurrentIndex(0)
        elif self.ui.comboBox_3_length.currentText() == "0.2":
            self.axis_3 = 0.2*100
            self.ui.comboBox_3_length.setCurrentIndex(1)
        elif self.ui.comboBox_3_length.currentText() == "0.25":
            self.axis_3 = 0.25*100
            self.ui.comboBox_3_length.setCurrentIndex(2)
        elif self.ui.comboBox_3_length.currentText() == "0.3":
            self.axis_3 = 0.3*100
            self.ui.comboBox_3_length.setCurrentIndex(3)
        elif self.ui.comboBox_3_length.currentText() == "0.35":
            self.axis_3 = 0.35*100
            self.ui.comboBox_3_length.setCurrentIndex(4)
        elif self.ui.comboBox_3_length.currentText() == "0.4":
            self.axis_3 = 0.4*100
            self.ui.comboBox_3_length.setCurrentIndex(5)
        elif self.ui.comboBox_3_length.currentText() == "0.45":
            self.axis_3 = 0.45*100
            self.ui.comboBox_3_length.setCurrentIndex(6)
        elif self.ui.comboBox_3_length.currentText() == "0.5":
            self.axis_3 = 0.5*100
            self.ui.comboBox_3_length.setCurrentIndex(7)

    def display_dof(self):
        if self.ui.comboBox_dof.currentText() == "1":
            self.dof = 1
            self.ui.comboBox_dof.setCurrentIndex(0)
        elif self.ui.comboBox_dof.currentText() == "2":
            self.dof = 2
            self.ui.comboBox_dof.setCurrentIndex(1)
        elif self.ui.comboBox_dof.currentText() == "3":
            self.dof = 3
            self.ui.comboBox_dof.setCurrentIndex(2)
        elif self.ui.comboBox_dof.currentText() == "4":
            self.dof = 4
            self.ui.comboBox_dof.setCurrentIndex(3)
        elif self.ui.comboBox_dof.currentText() == "5":
            self.dof = 5
            self.ui.comboBox_dof.setCurrentIndex(4)
        elif self.ui.comboBox_dof.currentText() == "6":
            self.dof = 6
            self.ui.comboBox_dof.setCurrentIndex(5)
        elif self.ui.comboBox_dof.currentText() == "7":
            self.dof = 7
            self.ui.comboBox_dof.setCurrentIndex(6)

    def display_dof_select(self):
        if self.ui.comboBox_dof_select.currentText() == "1":
            self.dof_select = 1
            self.ui.comboBox_dof_select.setCurrentIndex(0)
        elif self.ui.comboBox_dof_select.currentText() == "2":
            self.dof_select = 2
            self.ui.comboBox_dof_select.setCurrentIndex(1)
        elif self.ui.comboBox_dof_select.currentText() == "3":
            self.dof_select = 3
            self.ui.comboBox_dof_select.setCurrentIndex(2)
        elif self.ui.comboBox_dof_select.currentText() == "4":
            self.dof_select = 4
            self.ui.comboBox_dof_select.setCurrentIndex(3)
        elif self.ui.comboBox_dof_select.currentText() == "5":
            self.dof_select = 5
            self.ui.comboBox_dof_select.setCurrentIndex(4)
        elif self.ui.comboBox_dof_select.currentText() == "6":
            self.dof_select = 6
            self.ui.comboBox_dof_select.setCurrentIndex(5)
        elif self.ui.comboBox_dof_select.currentText() == "7":
            self.dof_select = 7
            self.ui.comboBox_dof_select.setCurrentIndex(6)
    
    # def display_structure_select(self):
    #     # ['random','TECO','UR','TM','Single Arm','SCARA','3DOF']
    #     if self.ui.comboBox_algorithm_select.currentText() == "random":
    #         self.algorithm_name = 'random'
    #         self.ui.comboBox_algorithm_select.setCurrentIndex(0)
    #     elif self.ui.comboBox_algorithm_select.currentText() == "TECO":
    #         self.algorithm_name = 'TECO'
    #         self.ui.comboBox_algorithm_select.setCurrentIndex(1)
    #     elif self.ui.comboBox_algorithm_select.currentText() == "UR":
    #         self.algorithm_name = 'UR'
    #         self.ui.comboBox_algorithm_select.setCurrentIndex(2)
    #     elif self.ui.comboBox_algorithm_select.currentText() == "TM":
    #         self.algorithm_name = 'TM'
    #         self.ui.comboBox_algorithm_select.setCurrentIndex(3)
    #     elif self.ui.comboBox_algorithm_select.currentText() == "Single Arm":
    #         self.algorithm_name = 'Single Arm'
    #         self.ui.comboBox_algorithm_select.setCurrentIndex(4)
    #     elif self.ui.comboBox_algorithm_select.currentText() == "SCARA":
    #         self.algorithm_name = 'SCARA'
    #         self.ui.comboBox_algorithm_select.setCurrentIndex(5)
    #     elif self.ui.comboBox_algorithm_select.currentText() == "3DOF":
    #         self.algorithm_name = '3DOF'
    #         self.ui.comboBox_algorithm_select.setCurrentIndex(6)
    def display_structure_select(self):
        if self.ui.comboBox_algorithm_select.currentText() == "DQN":
            self.algorithm_name = 'DQN'
            self.ui.comboBox_algorithm_select.setCurrentIndex(0)
        elif self.ui.comboBox_algorithm_select.currentText() == "DDQN":
            self.algorithm_name = 'DDQN'
            self.ui.comboBox_algorithm_select.setCurrentIndex(1)
        elif self.ui.comboBox_algorithm_select.currentText() == "C51":
            self.algorithm_name = 'C51'
            self.ui.comboBox_algorithm_select.setCurrentIndex(2)

    
    def dyna_set_buttonClicked(self):
        self.payload = float(self.ui.lineEdit_payload.text())
        self.dyna_data.payload = self.payload

        payload_x = float(self.ui.lineEdit_payload_x.text())
        payload_y = float(self.ui.lineEdit_payload_y.text())
        payload_z = float(self.ui.lineEdit_payload_z.text())
        self.payload_position = [payload_x, payload_y, payload_z]
        self.dyna_data.payload_position = self.payload_position

        vel_0 = float(self.ui.lineEdit_vel_0.text())
        vel_1 = float(self.ui.lineEdit_vel_1.text())
        vel_2 = float(self.ui.lineEdit_vel_2.text())
        vel_3 = float(self.ui.lineEdit_vel_3.text())
        vel_4 = float(self.ui.lineEdit_vel_4.text())
        vel_5 = float(self.ui.lineEdit_vel_5.text())
        vel_6 = float(self.ui.lineEdit_vel_6.text())
        self.joint_velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5, vel_6]
        self.dyna_data.vel = self.joint_velocity

        acc_0 = float(self.ui.lineEdit_acc_0.text())
        acc_1 = float(self.ui.lineEdit_acc_1.text())
        acc_2 = float(self.ui.lineEdit_acc_2.text())
        acc_3 = float(self.ui.lineEdit_acc_3.text())
        acc_4 = float(self.ui.lineEdit_acc_4.text())
        acc_5 = float(self.ui.lineEdit_acc_5.text())
        acc_6 = float(self.ui.lineEdit_acc_6.text())
        self.joint_acceleration = [acc_0, acc_1, acc_2, acc_3, acc_4, acc_5, acc_6]
        self.dyna_data.acc = self.joint_acceleration

        jog_0 = float(self.ui.lineEdit_jog_0.text())
        jog_1 = float(self.ui.lineEdit_jog_1.text())
        jog_2 = float(self.ui.lineEdit_jog_2.text())
        jog_3 = float(self.ui.lineEdit_jog_3.text())
        jog_4 = float(self.ui.lineEdit_jog_4.text())
        jog_5 = float(self.ui.lineEdit_jog_5.text())
        jog_6 = float(self.ui.lineEdit_jog_6.text())
        self.joint_angle = [jog_0, jog_1, jog_2, jog_3, jog_4, jog_5, jog_6]
        self.dyna_data.joint_angle = self.joint_angle

        self.pub_dyna_data.publish(self.dyna_data)

    def dyna_space_set_buttonClicked(self):
        # self.dyna_space_data
        self.payload_space = float(self.ui.lineEdit_payload_space.text())
        self.dyna_space_data.payload = self.payload_space

        payload_x = float(self.ui.lineEdit_payload_x_space.text())
        payload_y = float(self.ui.lineEdit_payload_y_space.text())
        payload_z = float(self.ui.lineEdit_payload_z_space.text())
        self.payload_position_space = [payload_x, payload_y, payload_z]
        self.dyna_space_data.payload_position = self.payload_position_space

        self.pub_dyna_space.publish(self.dyna_space_data)
        
    def axis_set_buttonClicked(self):
        self.axis = int(self.ui.lineEdit_axis_set.text())
        self.dyna_space_data.analysis_axis = self.axis
        self.pub_dyna_space.publish(self.dyna_space_data)

        self.pub_cmd.publish(3)

    def dyna_space_buttonClicked(self):
        self.pub_cmd.publish(1)

    def dyna_buttonClicked(self):
        self.pub_cmd.publish(2)

    def arm_plot_buttonClicked(self):
        self.pub_cmd.publish(4)

    def arm_plot_close_buttonClicked(self):
        self.pub_cmd.publish(5)
    
    def dyn_torque_limit_buttonClicked(self):
        self.pub_cmd.publish(6)
        
    def dynamics_design_buttonClicked(self):
        # arm_weight = float(self.ui.lineEdit_arm_weight.text())
        payload = float(self.ui.lineEdit_payload_2.text())
        reachable_radius = float(self.ui.lineEdit_reachable_radius.text())
        axis_2 = float(self.ui.lineEdit_2_length.text())
        axis_3 = float(self.ui.lineEdit_3_length.text())
        joint_limits = []
        joint_limits.append(float(self.ui.lineEdit_joint1.text()))
        joint_limits.append(float(self.ui.lineEdit_joint1_.text()))
        joint_limits.append(float(self.ui.lineEdit_joint2.text()))
        joint_limits.append(float(self.ui.lineEdit_joint2_.text()))
        joint_limits.append(float(self.ui.lineEdit_joint3.text()))
        joint_limits.append(float(self.ui.lineEdit_joint3_.text()))
        joint_limits.append(float(self.ui.lineEdit_joint4.text()))
        joint_limits.append(float(self.ui.lineEdit_joint4_.text()))
        joint_limits.append(float(self.ui.lineEdit_joint5.text()))
        joint_limits.append(float(self.ui.lineEdit_joint5_.text()))
        joint_limits.append(float(self.ui.lineEdit_joint6.text()))
        joint_limits.append(float(self.ui.lineEdit_joint6_.text()))
        joint_limits.append(float(self.ui.lineEdit_joint7.text()))
        joint_limits.append(float(self.ui.lineEdit_joint7_.text()))

        self.specified_parameter_design.axis_2_length = axis_2*100
        self.specified_parameter_design.axis_3_length = axis_3*100
        self.specified_parameter_design.arm_weight = 0
        self.specified_parameter_design.payload = payload
        self.specified_parameter_design.radius = reachable_radius
        self.specified_parameter_design.DoF = self.dof
        self.specified_parameter_design.joint_limit = joint_limits
        self.pub_specified_parameter_design.publish(self.specified_parameter_design)
        

    def dynamics_new_traj_buttonClicked(self):
        self.communicate_matlab.data = "1"
        self.pub_communicate_matlab.publish(self.communicate_matlab)

    def dynamics_real_torq_buttonClicked(self):
        pass

    def dynamics_save_para_buttonClicked(self):
        pass

    def random_robot_structure_buttonClicked(self):
        self.pub_cmd.publish(20)

    def random_robot_motor_buttonClicked(self):
        self.pub_cmd.publish(21)
    def optimization_set_buttonClicked(self):
        # input data: robot workspace, robot payload, robot joint velocity, robot joint acceleration
        
        payload = float(self.ui.lineEdit_op_payload.text())
        payload_x = float(self.ui.lineEdit_op_payload_x.text())
        payload_y = float(self.ui.lineEdit_op_payload_y.text())
        payload_z = float(self.ui.lineEdit_op_payload_z.text())
        payload_position = [payload_x, payload_y, payload_z]

        vel_0 = float(self.ui.lineEdit_op_vel_1.text())
        vel_1 = float(self.ui.lineEdit_op_vel_2.text())
        vel_2 = float(self.ui.lineEdit_op_vel_3.text())
        vel_3 = float(self.ui.lineEdit_op_vel_4.text())
        vel_4 = float(self.ui.lineEdit_op_vel_5.text())
        vel_5 = float(self.ui.lineEdit_op_vel_6.text())
        vel_6 = float(self.ui.lineEdit_op_vel_7.text())
        joint_velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5, vel_6]

        

        acc_0 = float(self.ui.lineEdit_op_acc_1.text())
        acc_1 = float(self.ui.lineEdit_op_acc_2.text())
        acc_2 = float(self.ui.lineEdit_op_acc_3.text())
        acc_3 = float(self.ui.lineEdit_op_acc_4.text())
        acc_4 = float(self.ui.lineEdit_op_acc_5.text())
        acc_5 = float(self.ui.lineEdit_op_acc_6.text())
        acc_6 = float(self.ui.lineEdit_op_acc_7.text())
        joint_acceleration = [acc_0, acc_1, acc_2, acc_3, acc_4, acc_5, acc_6]

        reachable_radius = float(self.ui.lineEdit_op_reachable_radius.text())

        # print ("payload: ", payload)
        # print ("payload_position: ", payload_position)
        # print ("joint_velocity: ", joint_velocity)
        # print ("joint_acceleration: ", joint_acceleration)
        # print ("Reachable radius: ", reachable_radius)

        # self.optimization_data.payload = payload
        self.optimal_design.payload = payload
        self.optimal_design.payload_position = payload_position
        self.optimal_design.dof = self.dof_select
        # self.optimal_design.structure = self.structure
        self.optimal_design.vel = joint_velocity
        self.optimal_design.acc = joint_acceleration
        self.optimal_design.radius = reachable_radius

        self.pub_optimal_design.publish(self.optimal_design)
        self.pub_cmd.publish(21)
        
    def optimization_analysis_buttonClicked(self):
        # input data: robot workspace, robot payload, robot joint velocity, robot joint acceleration
        
        payload = float(self.ui.lineEdit_op_payload.text())
        payload_x = float(self.ui.lineEdit_op_payload_x.text())
        payload_y = float(self.ui.lineEdit_op_payload_y.text())
        payload_z = float(self.ui.lineEdit_op_payload_z.text())
        payload_position = [payload_x, payload_y, payload_z]

        vel_0 = float(self.ui.lineEdit_op_vel_1.text())
        vel_1 = float(self.ui.lineEdit_op_vel_2.text())
        vel_2 = float(self.ui.lineEdit_op_vel_3.text())
        vel_3 = float(self.ui.lineEdit_op_vel_4.text())
        vel_4 = float(self.ui.lineEdit_op_vel_5.text())
        vel_5 = float(self.ui.lineEdit_op_vel_6.text())
        vel_6 = float(self.ui.lineEdit_op_vel_7.text())
        joint_velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5, vel_6]

        

        acc_0 = float(self.ui.lineEdit_op_acc_1.text())
        acc_1 = float(self.ui.lineEdit_op_acc_2.text())
        acc_2 = float(self.ui.lineEdit_op_acc_3.text())
        acc_3 = float(self.ui.lineEdit_op_acc_4.text())
        acc_4 = float(self.ui.lineEdit_op_acc_5.text())
        acc_5 = float(self.ui.lineEdit_op_acc_6.text())
        acc_6 = float(self.ui.lineEdit_op_acc_7.text())
        joint_acceleration = [acc_0, acc_1, acc_2, acc_3, acc_4, acc_5, acc_6]

        reachable_radius = float(self.ui.lineEdit_op_reachable_radius.text())

        weight =  float(self.ui.lineEdit_op_weight.text())
        cost = float(self.ui.lineEdit_op_cost.text())
        # print ("payload: ", payload)
        # print ("payload_position: ", payload_position)
        # print ("joint_velocity: ", joint_velocity)
        # print ("joint_acceleration: ", joint_acceleration)
        # print ("Reachable radius: ", reachable_radius)

        # self.optimization_data.payload = payload
        self.optimal_design.payload = payload
        self.optimal_design.payload_position = payload_position
        self.optimal_design.dof = self.dof_select
        # self.optimal_design.structure = self.structure
        self.optimal_design.vel = joint_velocity
        self.optimal_design.acc = joint_acceleration
        self.optimal_design.radius = reachable_radius
        self.optimal_design.arm_weight = weight
        self.optimal_design.cost = cost
        self.pub_optimal_design.publish(self.optimal_design)
        self.pub_cmd.publish(22)

    def cjm_select_buttonClicked(self):
        self.pub_cmd.publish(9)


    def traj_torque_plot_buttonClicked(self):
        self.pub_cmd.publish(10)
        
    def structure_set_buttonClicked(self):
        self.arm_structure.structure_name = self.algorithm_name
        self.arm_structure.DoF = self.dof_select
        self.pub_arm_structure.publish(self.arm_structure)
        
        
        # self.pub
    def btn_optimization_test_buttonClicked(self):
        # input data: robot workspace, robot payload, robot joint velocity, robot joint acceleration
        
        payload = float(self.ui.lineEdit_op_payload.text())
        payload_x = float(self.ui.lineEdit_op_payload_x.text())
        payload_y = float(self.ui.lineEdit_op_payload_y.text())
        payload_z = float(self.ui.lineEdit_op_payload_z.text())
        payload_position = [payload_x, payload_y, payload_z]

        vel_0 = float(self.ui.lineEdit_op_vel_1.text())
        vel_1 = float(self.ui.lineEdit_op_vel_2.text())
        vel_2 = float(self.ui.lineEdit_op_vel_3.text())
        vel_3 = float(self.ui.lineEdit_op_vel_4.text())
        vel_4 = float(self.ui.lineEdit_op_vel_5.text())
        vel_5 = float(self.ui.lineEdit_op_vel_6.text())
        vel_6 = float(self.ui.lineEdit_op_vel_7.text())
        joint_velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5, vel_6]

        

        acc_0 = float(self.ui.lineEdit_op_acc_1.text())
        acc_1 = float(self.ui.lineEdit_op_acc_2.text())
        acc_2 = float(self.ui.lineEdit_op_acc_3.text())
        acc_3 = float(self.ui.lineEdit_op_acc_4.text())
        acc_4 = float(self.ui.lineEdit_op_acc_5.text())
        acc_5 = float(self.ui.lineEdit_op_acc_6.text())
        acc_6 = float(self.ui.lineEdit_op_acc_7.text())
        joint_acceleration = [acc_0, acc_1, acc_2, acc_3, acc_4, acc_5, acc_6]

        reachable_radius = float(self.ui.lineEdit_op_reachable_radius.text())

        weight =  float(self.ui.lineEdit_op_weight.text())
        cost = float(self.ui.lineEdit_op_cost.text())
        # print ("payload: ", payload)
        # print ("payload_position: ", payload_position)
        # print ("joint_velocity: ", joint_velocity)
        # print ("joint_acceleration: ", joint_acceleration)
        # print ("Reachable radius: ", reachable_radius)

        # self.optimization_data.payload = payload
        self.optimal_design.payload = payload
        self.optimal_design.payload_position = payload_position
        self.optimal_design.dof = self.dof_select
        # self.optimal_design.structure = self.structure
        self.optimal_design.vel = joint_velocity
        self.optimal_design.acc = joint_acceleration
        self.optimal_design.radius = reachable_radius
        self.optimal_design.arm_weight = weight
        self.optimal_design.cost = cost

        test_model_name =  str(self.ui.lineEdit_op_test_model_name.text())
        self.tested_model_name.publish(test_model_name)
        self.pub_optimal_design.publish(self.optimal_design)
        self.pub_cmd.publish(23)

if __name__=="__main__":
    rospy.init_node("interface_ui")
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
    