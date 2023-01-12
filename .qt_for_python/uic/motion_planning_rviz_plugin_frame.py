# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'motion_planning_rviz_plugin_frame.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *



class Ui_MotionPlanningUI(object):
    def setupUi(self, MotionPlanningUI):
        if not MotionPlanningUI.objectName():
            MotionPlanningUI.setObjectName(u"MotionPlanningUI")
        MotionPlanningUI.resize(507, 380)
        self.verticalLayout_5 = QVBoxLayout(MotionPlanningUI)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.tabWidget = QTabWidget(MotionPlanningUI)
        self.tabWidget.setObjectName(u"tabWidget")
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tabWidget.sizePolicy().hasHeightForWidth())
        self.tabWidget.setSizePolicy(sizePolicy)
        self.tabWidget.setAutoFillBackground(False)
        self.context = QWidget()
        self.context.setObjectName(u"context")
        self.verticalLayout_3 = QVBoxLayout(self.context)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.groupBox1 = QGroupBox(self.context)
        self.groupBox1.setObjectName(u"groupBox1")
        self.horizontalLayout_8 = QHBoxLayout(self.groupBox1)
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.horizontalLayout_8.setContentsMargins(-1, -1, -1, 1)
        self.verticalLayout_13 = QVBoxLayout()
        self.verticalLayout_13.setObjectName(u"verticalLayout_13")
        self.library_label = QLabel(self.groupBox1)
        self.library_label.setObjectName(u"library_label")
        sizePolicy1 = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.library_label.sizePolicy().hasHeightForWidth())
        self.library_label.setSizePolicy(sizePolicy1)

        self.verticalLayout_13.addWidget(self.library_label)

        self.planning_pipeline_combo_box = QComboBox(self.groupBox1)
        self.planning_pipeline_combo_box.setObjectName(u"planning_pipeline_combo_box")
        sizePolicy1.setHeightForWidth(self.planning_pipeline_combo_box.sizePolicy().hasHeightForWidth())
        self.planning_pipeline_combo_box.setSizePolicy(sizePolicy1)

        self.verticalLayout_13.addWidget(self.planning_pipeline_combo_box)

        self.planning_algorithm_combo_box = QComboBox(self.groupBox1)
        self.planning_algorithm_combo_box.setObjectName(u"planning_algorithm_combo_box")
        sizePolicy1.setHeightForWidth(self.planning_algorithm_combo_box.sizePolicy().hasHeightForWidth())
        self.planning_algorithm_combo_box.setSizePolicy(sizePolicy1)

        self.verticalLayout_13.addWidget(self.planning_algorithm_combo_box)

        self.verticalSpacer_10 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_13.addItem(self.verticalSpacer_10)


        self.horizontalLayout_8.addLayout(self.verticalLayout_13)

        self.verticalLayout_14 = QVBoxLayout()
        self.verticalLayout_14.setObjectName(u"verticalLayout_14")
        self.label_15 = QLabel(self.groupBox1)
        self.label_15.setObjectName(u"label_15")

        self.verticalLayout_14.addWidget(self.label_15)

        self.planner_param_treeview = moveit_rviz_plugin_MotionPlanningParamWidget(self.groupBox1)
        self.planner_param_treeview.setObjectName(u"planner_param_treeview")
        self.planner_param_treeview.setAutoScroll(False)
        self.planner_param_treeview.setIndentation(0)
        self.planner_param_treeview.setRootIsDecorated(False)
        self.planner_param_treeview.header().setMinimumSectionSize(20)

        self.verticalLayout_14.addWidget(self.planner_param_treeview)


        self.horizontalLayout_8.addLayout(self.verticalLayout_14)

        self.horizontalLayout_8.setStretch(0, 2)
        self.horizontalLayout_8.setStretch(1, 3)

        self.verticalLayout_3.addWidget(self.groupBox1)

        self.groupBox2 = QGroupBox(self.context)
        self.groupBox2.setObjectName(u"groupBox2")
        self.horizontalLayout = QHBoxLayout(self.groupBox2)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(-1, 4, -1, 1)
        self.label_2 = QLabel(self.groupBox2)
        self.label_2.setObjectName(u"label_2")

        self.horizontalLayout.addWidget(self.label_2)

        self.database_host = QLineEdit(self.groupBox2)
        self.database_host.setObjectName(u"database_host")

        self.horizontalLayout.addWidget(self.database_host)

        self.label_3 = QLabel(self.groupBox2)
        self.label_3.setObjectName(u"label_3")

        self.horizontalLayout.addWidget(self.label_3)

        self.database_port = QSpinBox(self.groupBox2)
        self.database_port.setObjectName(u"database_port")
        self.database_port.setMaximum(65535)
        self.database_port.setValue(0)

        self.horizontalLayout.addWidget(self.database_port)

        self.reset_db_button = QPushButton(self.groupBox2)
        self.reset_db_button.setObjectName(u"reset_db_button")
        sizePolicy2 = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.reset_db_button.sizePolicy().hasHeightForWidth())
        self.reset_db_button.setSizePolicy(sizePolicy2)
        self.reset_db_button.setStyleSheet(u"color:red")
        self.reset_db_button.setFlat(False)

        self.horizontalLayout.addWidget(self.reset_db_button)

        self.database_connect_button = QPushButton(self.groupBox2)
        self.database_connect_button.setObjectName(u"database_connect_button")
        self.database_connect_button.setStyleSheet(u"color:green")
        self.database_connect_button.setCheckable(False)
        self.database_connect_button.setChecked(False)
        self.database_connect_button.setFlat(False)

        self.horizontalLayout.addWidget(self.database_connect_button)


        self.verticalLayout_3.addWidget(self.groupBox2)

        self.groupBox_6 = QGroupBox(self.context)
        self.groupBox_6.setObjectName(u"groupBox_6")
        self.gridLayout_18 = QGridLayout(self.groupBox_6)
        self.gridLayout_18.setObjectName(u"gridLayout_18")
        self.gridLayout_18.setContentsMargins(-1, 4, -1, 1)
        self.label_21 = QLabel(self.groupBox_6)
        self.label_21.setObjectName(u"label_21")

        self.gridLayout_18.addWidget(self.label_21, 0, 0, 1, 1)

        self.wcenter_x = QDoubleSpinBox(self.groupBox_6)
        self.wcenter_x.setObjectName(u"wcenter_x")
        sizePolicy3 = QSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.wcenter_x.sizePolicy().hasHeightForWidth())
        self.wcenter_x.setSizePolicy(sizePolicy3)
        self.wcenter_x.setMinimum(-10000.000000000000000)
        self.wcenter_x.setMaximum(10000.000000000000000)

        self.gridLayout_18.addWidget(self.wcenter_x, 0, 1, 1, 1)

        self.wcenter_y = QDoubleSpinBox(self.groupBox_6)
        self.wcenter_y.setObjectName(u"wcenter_y")
        sizePolicy3.setHeightForWidth(self.wcenter_y.sizePolicy().hasHeightForWidth())
        self.wcenter_y.setSizePolicy(sizePolicy3)
        self.wcenter_y.setMinimum(-10000.000000000000000)
        self.wcenter_y.setMaximum(10000.000000000000000)

        self.gridLayout_18.addWidget(self.wcenter_y, 0, 2, 1, 1)

        self.wcenter_z = QDoubleSpinBox(self.groupBox_6)
        self.wcenter_z.setObjectName(u"wcenter_z")
        sizePolicy3.setHeightForWidth(self.wcenter_z.sizePolicy().hasHeightForWidth())
        self.wcenter_z.setSizePolicy(sizePolicy3)
        self.wcenter_z.setMinimum(-10000.000000000000000)
        self.wcenter_z.setMaximum(10000.000000000000000)

        self.gridLayout_18.addWidget(self.wcenter_z, 0, 3, 1, 1)

        self.label_22 = QLabel(self.groupBox_6)
        self.label_22.setObjectName(u"label_22")

        self.gridLayout_18.addWidget(self.label_22, 1, 0, 1, 1)

        self.wsize_x = QDoubleSpinBox(self.groupBox_6)
        self.wsize_x.setObjectName(u"wsize_x")
        sizePolicy3.setHeightForWidth(self.wsize_x.sizePolicy().hasHeightForWidth())
        self.wsize_x.setSizePolicy(sizePolicy3)
        self.wsize_x.setWrapping(False)
        self.wsize_x.setMinimum(-10000.000000000000000)
        self.wsize_x.setMaximum(10000.000000000000000)
        self.wsize_x.setValue(2.000000000000000)

        self.gridLayout_18.addWidget(self.wsize_x, 1, 1, 1, 1)

        self.wsize_y = QDoubleSpinBox(self.groupBox_6)
        self.wsize_y.setObjectName(u"wsize_y")
        sizePolicy3.setHeightForWidth(self.wsize_y.sizePolicy().hasHeightForWidth())
        self.wsize_y.setSizePolicy(sizePolicy3)
        self.wsize_y.setMinimum(-10000.000000000000000)
        self.wsize_y.setMaximum(10000.000000000000000)
        self.wsize_y.setValue(2.000000000000000)

        self.gridLayout_18.addWidget(self.wsize_y, 1, 2, 1, 1)

        self.wsize_z = QDoubleSpinBox(self.groupBox_6)
        self.wsize_z.setObjectName(u"wsize_z")
        sizePolicy3.setHeightForWidth(self.wsize_z.sizePolicy().hasHeightForWidth())
        self.wsize_z.setSizePolicy(sizePolicy3)
        self.wsize_z.setMinimum(-10000.000000000000000)
        self.wsize_z.setMaximum(10000.000000000000000)
        self.wsize_z.setValue(2.000000000000000)

        self.gridLayout_18.addWidget(self.wsize_z, 1, 3, 1, 1)


        self.verticalLayout_3.addWidget(self.groupBox_6)

        self.verticalSpacer_5 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_3.addItem(self.verticalSpacer_5)

        self.tabWidget.addTab(self.context, "")
        self.planning = QWidget()
        self.planning.setObjectName(u"planning")
        self.horizontalLayout_9 = QHBoxLayout(self.planning)
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.verticalLayout_11 = QVBoxLayout()
        self.verticalLayout_11.setSpacing(0)
        self.verticalLayout_11.setObjectName(u"verticalLayout_11")
        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setSpacing(0)
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.groupBox_9 = QGroupBox(self.planning)
        self.groupBox_9.setObjectName(u"groupBox_9")
        self.verticalLayout_9 = QVBoxLayout(self.groupBox_9)
        self.verticalLayout_9.setSpacing(4)
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.verticalLayout_9.setContentsMargins(6, -1, 6, 6)
        self.plan_button = QPushButton(self.groupBox_9)
        self.plan_button.setObjectName(u"plan_button")

        self.verticalLayout_9.addWidget(self.plan_button)

        self.execute_button = QPushButton(self.groupBox_9)
        self.execute_button.setObjectName(u"execute_button")
        self.execute_button.setEnabled(False)

        self.verticalLayout_9.addWidget(self.execute_button)

        self.plan_and_execute_button = QPushButton(self.groupBox_9)
        self.plan_and_execute_button.setObjectName(u"plan_and_execute_button")

        self.verticalLayout_9.addWidget(self.plan_and_execute_button)

        self.stop_button = QPushButton(self.groupBox_9)
        self.stop_button.setObjectName(u"stop_button")
        self.stop_button.setEnabled(False)

        self.verticalLayout_9.addWidget(self.stop_button)

        self.result_label = QLabel(self.groupBox_9)
        self.result_label.setObjectName(u"result_label")
        self.result_label.setLayoutDirection(Qt.LeftToRight)
        self.result_label.setAlignment(Qt.AlignCenter)
        self.result_label.setWordWrap(False)

        self.verticalLayout_9.addWidget(self.result_label)

        self.clear_octomap_button = QPushButton(self.groupBox_9)
        self.clear_octomap_button.setObjectName(u"clear_octomap_button")
        self.clear_octomap_button.setEnabled(False)

        self.verticalLayout_9.addWidget(self.clear_octomap_button)


        self.horizontalLayout_7.addWidget(self.groupBox_9)

        self.groupBox_11 = QGroupBox(self.planning)
        self.groupBox_11.setObjectName(u"groupBox_11")
        self.verticalLayout_6 = QVBoxLayout(self.groupBox_11)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.verticalLayout_6.setContentsMargins(6, -1, 6, 6)
        self.verticalLayout_planning_group = QVBoxLayout()
        self.verticalLayout_planning_group.setObjectName(u"verticalLayout_planning_group")
        self.label_planning_group = QLabel(self.groupBox_11)
        self.label_planning_group.setObjectName(u"label_planning_group")

        self.verticalLayout_planning_group.addWidget(self.label_planning_group)

        self.planning_group_combo_box = QComboBox(self.groupBox_11)
        self.planning_group_combo_box.setObjectName(u"planning_group_combo_box")

        self.verticalLayout_planning_group.addWidget(self.planning_group_combo_box)


        self.verticalLayout_6.addLayout(self.verticalLayout_planning_group)

        self.verticalLayout_start_state = QVBoxLayout()
        self.verticalLayout_start_state.setObjectName(u"verticalLayout_start_state")
        self.label_start_state = QLabel(self.groupBox_11)
        self.label_start_state.setObjectName(u"label_start_state")

        self.verticalLayout_start_state.addWidget(self.label_start_state)

        self.start_state_combo_box = QComboBox(self.groupBox_11)
        self.start_state_combo_box.setObjectName(u"start_state_combo_box")

        self.verticalLayout_start_state.addWidget(self.start_state_combo_box)


        self.verticalLayout_6.addLayout(self.verticalLayout_start_state)

        self.verticalLayout_goal_state = QVBoxLayout()
        self.verticalLayout_goal_state.setObjectName(u"verticalLayout_goal_state")
        self.label_goal_state = QLabel(self.groupBox_11)
        self.label_goal_state.setObjectName(u"label_goal_state")

        self.verticalLayout_goal_state.addWidget(self.label_goal_state)

        self.goal_state_combo_box = QComboBox(self.groupBox_11)
        self.goal_state_combo_box.setObjectName(u"goal_state_combo_box")

        self.verticalLayout_goal_state.addWidget(self.goal_state_combo_box)


        self.verticalLayout_6.addLayout(self.verticalLayout_goal_state)


        self.horizontalLayout_7.addWidget(self.groupBox_11)


        self.verticalLayout_11.addLayout(self.horizontalLayout_7)

        self.verticalSpacer = QSpacerItem(20, 32, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_11.addItem(self.verticalSpacer)

        self.groupBox_12 = QGroupBox(self.planning)
        self.groupBox_12.setObjectName(u"groupBox_12")
        self.verticalLayout_15 = QVBoxLayout(self.groupBox_12)
        self.verticalLayout_15.setObjectName(u"verticalLayout_15")
        self.verticalLayout_15.setContentsMargins(6, 4, 6, 6)
        self.path_constraints_combo_box = QComboBox(self.groupBox_12)
        self.path_constraints_combo_box.setObjectName(u"path_constraints_combo_box")

        self.verticalLayout_15.addWidget(self.path_constraints_combo_box)


        self.verticalLayout_11.addWidget(self.groupBox_12)


        self.horizontalLayout_9.addLayout(self.verticalLayout_11)

        self.verticalLayout_18 = QVBoxLayout()
        self.verticalLayout_18.setSpacing(0)
        self.verticalLayout_18.setObjectName(u"verticalLayout_18")
        self.groupBox_10 = QGroupBox(self.planning)
        self.groupBox_10.setObjectName(u"groupBox_10")
        self.verticalLayout_10 = QVBoxLayout(self.groupBox_10)
        self.verticalLayout_10.setSpacing(1)
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.verticalLayout_10.setContentsMargins(-1, -1, -1, 4)
        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setSpacing(1)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.label_9 = QLabel(self.groupBox_10)
        self.label_9.setObjectName(u"label_9")

        self.horizontalLayout_3.addWidget(self.label_9)

        self.planning_time = QDoubleSpinBox(self.groupBox_10)
        self.planning_time.setObjectName(u"planning_time")
        sizePolicy4 = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
        sizePolicy4.setHorizontalStretch(0)
        sizePolicy4.setVerticalStretch(0)
        sizePolicy4.setHeightForWidth(self.planning_time.sizePolicy().hasHeightForWidth())
        self.planning_time.setSizePolicy(sizePolicy4)
        self.planning_time.setMinimumSize(QSize(40, 0))
        self.planning_time.setDecimals(1)
        self.planning_time.setMaximum(300.000000000000000)
        self.planning_time.setValue(5.000000000000000)

        self.horizontalLayout_3.addWidget(self.planning_time)


        self.verticalLayout_10.addLayout(self.horizontalLayout_3)

        self.horizontalLayout_15 = QHBoxLayout()
        self.horizontalLayout_15.setSpacing(1)
        self.horizontalLayout_15.setObjectName(u"horizontalLayout_15")
        self.label_10 = QLabel(self.groupBox_10)
        self.label_10.setObjectName(u"label_10")

        self.horizontalLayout_15.addWidget(self.label_10)

        self.planning_attempts = QSpinBox(self.groupBox_10)
        self.planning_attempts.setObjectName(u"planning_attempts")
        sizePolicy4.setHeightForWidth(self.planning_attempts.sizePolicy().hasHeightForWidth())
        self.planning_attempts.setSizePolicy(sizePolicy4)
        self.planning_attempts.setMinimumSize(QSize(40, 0))
        self.planning_attempts.setMaximum(1000)
        self.planning_attempts.setValue(10)

        self.horizontalLayout_15.addWidget(self.planning_attempts)


        self.verticalLayout_10.addLayout(self.horizontalLayout_15)

        self.horizontalLayout_10 = QHBoxLayout()
        self.horizontalLayout_10.setSpacing(1)
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.label_11 = QLabel(self.groupBox_10)
        self.label_11.setObjectName(u"label_11")

        self.horizontalLayout_10.addWidget(self.label_11)

        self.velocity_scaling_factor = QDoubleSpinBox(self.groupBox_10)
        self.velocity_scaling_factor.setObjectName(u"velocity_scaling_factor")
        self.velocity_scaling_factor.setMinimumSize(QSize(40, 0))
        self.velocity_scaling_factor.setMaximum(1.000000000000000)
        self.velocity_scaling_factor.setSingleStep(0.100000000000000)
        self.velocity_scaling_factor.setValue(1.000000000000000)

        self.horizontalLayout_10.addWidget(self.velocity_scaling_factor)


        self.verticalLayout_10.addLayout(self.horizontalLayout_10)

        self.horizontalLayout_20 = QHBoxLayout()
        self.horizontalLayout_20.setSpacing(1)
        self.horizontalLayout_20.setObjectName(u"horizontalLayout_20")
        self.label_20 = QLabel(self.groupBox_10)
        self.label_20.setObjectName(u"label_20")

        self.horizontalLayout_20.addWidget(self.label_20)

        self.acceleration_scaling_factor = QDoubleSpinBox(self.groupBox_10)
        self.acceleration_scaling_factor.setObjectName(u"acceleration_scaling_factor")
        self.acceleration_scaling_factor.setMinimumSize(QSize(40, 0))
        self.acceleration_scaling_factor.setMaximum(1.000000000000000)
        self.acceleration_scaling_factor.setSingleStep(0.100000000000000)
        self.acceleration_scaling_factor.setValue(1.000000000000000)

        self.horizontalLayout_20.addWidget(self.acceleration_scaling_factor)


        self.verticalLayout_10.addLayout(self.horizontalLayout_20)

        self.verticalSpacer_2 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_10.addItem(self.verticalSpacer_2)

        self.use_cartesian_path = QCheckBox(self.groupBox_10)
        self.use_cartesian_path.setObjectName(u"use_cartesian_path")
        sizePolicy2.setHeightForWidth(self.use_cartesian_path.sizePolicy().hasHeightForWidth())
        self.use_cartesian_path.setSizePolicy(sizePolicy2)
        self.use_cartesian_path.setMinimumSize(QSize(30, 0))
        self.use_cartesian_path.setChecked(False)

        self.verticalLayout_10.addWidget(self.use_cartesian_path)

        self.collision_aware_ik = QCheckBox(self.groupBox_10)
        self.collision_aware_ik.setObjectName(u"collision_aware_ik")
        sizePolicy2.setHeightForWidth(self.collision_aware_ik.sizePolicy().hasHeightForWidth())
        self.collision_aware_ik.setSizePolicy(sizePolicy2)
        self.collision_aware_ik.setMinimumSize(QSize(30, 0))
        self.collision_aware_ik.setChecked(False)

        self.verticalLayout_10.addWidget(self.collision_aware_ik)

        self.approximate_ik = QCheckBox(self.groupBox_10)
        self.approximate_ik.setObjectName(u"approximate_ik")
        sizePolicy2.setHeightForWidth(self.approximate_ik.sizePolicy().hasHeightForWidth())
        self.approximate_ik.setSizePolicy(sizePolicy2)
        self.approximate_ik.setMinimumSize(QSize(30, 0))

        self.verticalLayout_10.addWidget(self.approximate_ik)

        self.allow_external_program = QCheckBox(self.groupBox_10)
        self.allow_external_program.setObjectName(u"allow_external_program")
        sizePolicy2.setHeightForWidth(self.allow_external_program.sizePolicy().hasHeightForWidth())
        self.allow_external_program.setSizePolicy(sizePolicy2)
        self.allow_external_program.setMinimumSize(QSize(30, 0))
        self.allow_external_program.setChecked(False)

        self.verticalLayout_10.addWidget(self.allow_external_program)

        self.allow_replanning = QCheckBox(self.groupBox_10)
        self.allow_replanning.setObjectName(u"allow_replanning")
        sizePolicy2.setHeightForWidth(self.allow_replanning.sizePolicy().hasHeightForWidth())
        self.allow_replanning.setSizePolicy(sizePolicy2)
        self.allow_replanning.setMinimumSize(QSize(30, 0))
        self.allow_replanning.setAutoFillBackground(False)
        self.allow_replanning.setChecked(False)

        self.verticalLayout_10.addWidget(self.allow_replanning)

        self.allow_looking = QCheckBox(self.groupBox_10)
        self.allow_looking.setObjectName(u"allow_looking")
        sizePolicy2.setHeightForWidth(self.allow_looking.sizePolicy().hasHeightForWidth())
        self.allow_looking.setSizePolicy(sizePolicy2)
        self.allow_looking.setMinimumSize(QSize(30, 0))
        self.allow_looking.setChecked(False)

        self.verticalLayout_10.addWidget(self.allow_looking)


        self.verticalLayout_18.addWidget(self.groupBox_10)


        self.horizontalLayout_9.addLayout(self.verticalLayout_18)

        self.tabWidget.addTab(self.planning, "")
        self.scene_collision_objects = QWidget()
        self.scene_collision_objects.setObjectName(u"scene_collision_objects")
        self.gridLayout_4 = QHBoxLayout(self.scene_collision_objects)
        self.gridLayout_4.setObjectName(u"gridLayout_4")
        self.verticalLayout_19 = QVBoxLayout()
        self.verticalLayout_19.setObjectName(u"verticalLayout_19")
        self.groupBox_8 = QGroupBox(self.scene_collision_objects)
        self.groupBox_8.setObjectName(u"groupBox_8")
        self.verticalLayout_17 = QVBoxLayout(self.groupBox_8)
        self.verticalLayout_17.setObjectName(u"verticalLayout_17")
        self.verticalLayout_17.setContentsMargins(6, 6, 6, 6)
        self.collision_objects_list = QListWidget(self.groupBox_8)
        self.collision_objects_list.setObjectName(u"collision_objects_list")
        sizePolicy.setHeightForWidth(self.collision_objects_list.sizePolicy().hasHeightForWidth())
        self.collision_objects_list.setSizePolicy(sizePolicy)
        self.collision_objects_list.setEditTriggers(QAbstractItemView.DoubleClicked|QAbstractItemView.EditKeyPressed)

        self.verticalLayout_17.addWidget(self.collision_objects_list)


        self.verticalLayout_19.addWidget(self.groupBox_8)

        self.groupBox_4 = QGroupBox(self.scene_collision_objects)
        self.groupBox_4.setObjectName(u"groupBox_4")
        sizePolicy5 = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy5.setHorizontalStretch(0)
        sizePolicy5.setVerticalStretch(0)
        sizePolicy5.setHeightForWidth(self.groupBox_4.sizePolicy().hasHeightForWidth())
        self.groupBox_4.setSizePolicy(sizePolicy5)
        self.verticalLayout_16 = QVBoxLayout(self.groupBox_4)
        self.verticalLayout_16.setObjectName(u"verticalLayout_16")
        self.verticalLayout_16.setContentsMargins(6, 6, 6, 6)
        self.horizontalLayout_16 = QHBoxLayout()
        self.horizontalLayout_16.setObjectName(u"horizontalLayout_16")
        self.shape_size_x_spin_box = QDoubleSpinBox(self.groupBox_4)
        self.shape_size_x_spin_box.setObjectName(u"shape_size_x_spin_box")
        sizePolicy2.setHeightForWidth(self.shape_size_x_spin_box.sizePolicy().hasHeightForWidth())
        self.shape_size_x_spin_box.setSizePolicy(sizePolicy2)
        self.shape_size_x_spin_box.setMinimumSize(QSize(20, 0))
        self.shape_size_x_spin_box.setSingleStep(0.050000000000000)
        self.shape_size_x_spin_box.setValue(0.200000000000000)

        self.horizontalLayout_16.addWidget(self.shape_size_x_spin_box)

        self.shape_size_y_spin_box = QDoubleSpinBox(self.groupBox_4)
        self.shape_size_y_spin_box.setObjectName(u"shape_size_y_spin_box")
        sizePolicy2.setHeightForWidth(self.shape_size_y_spin_box.sizePolicy().hasHeightForWidth())
        self.shape_size_y_spin_box.setSizePolicy(sizePolicy2)
        self.shape_size_y_spin_box.setMinimumSize(QSize(20, 0))
        self.shape_size_y_spin_box.setSingleStep(0.050000000000000)
        self.shape_size_y_spin_box.setValue(0.200000000000000)

        self.horizontalLayout_16.addWidget(self.shape_size_y_spin_box)

        self.shape_size_z_spin_box = QDoubleSpinBox(self.groupBox_4)
        self.shape_size_z_spin_box.setObjectName(u"shape_size_z_spin_box")
        sizePolicy2.setHeightForWidth(self.shape_size_z_spin_box.sizePolicy().hasHeightForWidth())
        self.shape_size_z_spin_box.setSizePolicy(sizePolicy2)
        self.shape_size_z_spin_box.setMinimumSize(QSize(20, 0))
        self.shape_size_z_spin_box.setSingleStep(0.050000000000000)
        self.shape_size_z_spin_box.setValue(0.200000000000000)

        self.horizontalLayout_16.addWidget(self.shape_size_z_spin_box)


        self.verticalLayout_16.addLayout(self.horizontalLayout_16)

        self.horizontalLayout_13 = QHBoxLayout()
        self.horizontalLayout_13.setObjectName(u"horizontalLayout_13")
        self.shapes_combo_box = QComboBox(self.groupBox_4)
        self.shapes_combo_box.setObjectName(u"shapes_combo_box")
        self.shapes_combo_box.setMinimumSize(QSize(30, 0))

        self.horizontalLayout_13.addWidget(self.shapes_combo_box)

        self.add_object_button = QToolButton(self.groupBox_4)
        self.add_object_button.setObjectName(u"add_object_button")
        sizePolicy6 = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        sizePolicy6.setHorizontalStretch(0)
        sizePolicy6.setVerticalStretch(0)
        sizePolicy6.setHeightForWidth(self.add_object_button.sizePolicy().hasHeightForWidth())
        self.add_object_button.setSizePolicy(sizePolicy6)
        icon = QIcon()
        icon.addFile(u":/icons/list-add.png", QSize(), QIcon.Normal, QIcon.Off)
        self.add_object_button.setIcon(icon)

        self.horizontalLayout_13.addWidget(self.add_object_button)

        self.remove_object_button = QToolButton(self.groupBox_4)
        self.remove_object_button.setObjectName(u"remove_object_button")
        sizePolicy6.setHeightForWidth(self.remove_object_button.sizePolicy().hasHeightForWidth())
        self.remove_object_button.setSizePolicy(sizePolicy6)
        icon1 = QIcon()
        icon1.addFile(u":/icons/list-remove.png", QSize(), QIcon.Normal, QIcon.Off)
        self.remove_object_button.setIcon(icon1)

        self.horizontalLayout_13.addWidget(self.remove_object_button)

        self.clear_scene_button = QToolButton(self.groupBox_4)
        self.clear_scene_button.setObjectName(u"clear_scene_button")
        sizePolicy6.setHeightForWidth(self.clear_scene_button.sizePolicy().hasHeightForWidth())
        self.clear_scene_button.setSizePolicy(sizePolicy6)
        icon2 = QIcon()
        icon2.addFile(u":/icons/edit-clear.png", QSize(), QIcon.Normal, QIcon.Off)
        self.clear_scene_button.setIcon(icon2)

        self.horizontalLayout_13.addWidget(self.clear_scene_button)


        self.verticalLayout_16.addLayout(self.horizontalLayout_13)


        self.verticalLayout_19.addWidget(self.groupBox_4)


        self.gridLayout_4.addLayout(self.verticalLayout_19)

        self.verticalLayout_20 = QVBoxLayout()
        self.verticalLayout_20.setObjectName(u"verticalLayout_20")
        self.pose_scale_group_box = QGroupBox(self.scene_collision_objects)
        self.pose_scale_group_box.setObjectName(u"pose_scale_group_box")
        self.gridLayout = QGridLayout(self.pose_scale_group_box)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setVerticalSpacing(6)
        self.gridLayout.setContentsMargins(6, 6, 6, 6)
        self.label_4 = QLabel(self.pose_scale_group_box)
        self.label_4.setObjectName(u"label_4")

        self.gridLayout.addWidget(self.label_4, 1, 0, 1, 1)

        self.label_13 = QLabel(self.pose_scale_group_box)
        self.label_13.setObjectName(u"label_13")

        self.gridLayout.addWidget(self.label_13, 2, 0, 1, 1)

        self.object_x = QDoubleSpinBox(self.pose_scale_group_box)
        self.object_x.setObjectName(u"object_x")
        self.object_x.setMinimumSize(QSize(20, 0))
        self.object_x.setMinimum(-999.990000000000009)
        self.object_x.setMaximum(999.990000000000009)
        self.object_x.setSingleStep(0.100000000000000)

        self.gridLayout.addWidget(self.object_x, 1, 1, 1, 1)

        self.object_rz = QDoubleSpinBox(self.pose_scale_group_box)
        self.object_rz.setObjectName(u"object_rz")
        self.object_rz.setMinimumSize(QSize(20, 0))
        self.object_rz.setMinimum(-3.140000000000000)
        self.object_rz.setMaximum(3.140000000000000)
        self.object_rz.setSingleStep(0.100000000000000)

        self.gridLayout.addWidget(self.object_rz, 2, 3, 1, 1)

        self.object_rx = QDoubleSpinBox(self.pose_scale_group_box)
        self.object_rx.setObjectName(u"object_rx")
        self.object_rx.setMinimumSize(QSize(20, 0))
        self.object_rx.setMinimum(-3.140000000000000)
        self.object_rx.setMaximum(3.140000000000000)
        self.object_rx.setSingleStep(0.100000000000000)

        self.gridLayout.addWidget(self.object_rx, 2, 1, 1, 1)

        self.object_ry = QDoubleSpinBox(self.pose_scale_group_box)
        self.object_ry.setObjectName(u"object_ry")
        self.object_ry.setMinimumSize(QSize(20, 0))
        self.object_ry.setMinimum(-3.140000000000000)
        self.object_ry.setMaximum(3.140000000000000)
        self.object_ry.setSingleStep(0.100000000000000)

        self.gridLayout.addWidget(self.object_ry, 2, 2, 1, 1)

        self.object_y = QDoubleSpinBox(self.pose_scale_group_box)
        self.object_y.setObjectName(u"object_y")
        self.object_y.setMinimumSize(QSize(20, 0))
        self.object_y.setMinimum(-999.990000000000009)
        self.object_y.setMaximum(999.990000000000009)
        self.object_y.setSingleStep(0.100000000000000)

        self.gridLayout.addWidget(self.object_y, 1, 2, 1, 1)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.label = QLabel(self.pose_scale_group_box)
        self.label.setObjectName(u"label")

        self.horizontalLayout_5.addWidget(self.label)

        self.label_17 = QLabel(self.pose_scale_group_box)
        self.label_17.setObjectName(u"label_17")

        self.horizontalLayout_5.addWidget(self.label_17)

        self.scene_scale = QSlider(self.pose_scale_group_box)
        self.scene_scale.setObjectName(u"scene_scale")
        self.scene_scale.setMinimumSize(QSize(40, 0))
        self.scene_scale.setMaximum(200)
        self.scene_scale.setValue(100)
        self.scene_scale.setSliderPosition(100)
        self.scene_scale.setOrientation(Qt.Horizontal)
        self.scene_scale.setInvertedAppearance(False)
        self.scene_scale.setTickPosition(QSlider.NoTicks)
        self.scene_scale.setTickInterval(0)

        self.horizontalLayout_5.addWidget(self.scene_scale)

        self.label_18 = QLabel(self.pose_scale_group_box)
        self.label_18.setObjectName(u"label_18")

        self.horizontalLayout_5.addWidget(self.label_18)


        self.gridLayout.addLayout(self.horizontalLayout_5, 3, 0, 1, 4)

        self.object_z = QDoubleSpinBox(self.pose_scale_group_box)
        self.object_z.setObjectName(u"object_z")
        self.object_z.setMinimumSize(QSize(20, 0))
        self.object_z.setMinimum(-999.990000000000009)
        self.object_z.setMaximum(999.990000000000009)
        self.object_z.setSingleStep(0.100000000000000)

        self.gridLayout.addWidget(self.object_z, 1, 3, 1, 1)


        self.verticalLayout_20.addWidget(self.pose_scale_group_box)

        self.groupBox_5 = QGroupBox(self.scene_collision_objects)
        self.groupBox_5.setObjectName(u"groupBox_5")
        self.verticalLayout_12 = QVBoxLayout(self.groupBox_5)
        self.verticalLayout_12.setObjectName(u"verticalLayout_12")
        self.verticalLayout_12.setContentsMargins(6, 6, 6, 6)
        self.object_status = QLabel(self.groupBox_5)
        self.object_status.setObjectName(u"object_status")
        self.object_status.setEnabled(True)
        self.object_status.setFrameShape(QFrame.NoFrame)
        self.object_status.setFrameShadow(QFrame.Raised)
        self.object_status.setTextFormat(Qt.AutoText)
        self.object_status.setWordWrap(True)
        self.object_status.setProperty("WordWrap", True)

        self.verticalLayout_12.addWidget(self.object_status)


        self.verticalLayout_20.addWidget(self.groupBox_5)

        self.verticalSpacer_4 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_20.addItem(self.verticalSpacer_4)

        self.groupBox3 = QGroupBox(self.scene_collision_objects)
        self.groupBox3.setObjectName(u"groupBox3")
        self.horizontalLayout_14 = QHBoxLayout(self.groupBox3)
        self.horizontalLayout_14.setObjectName(u"horizontalLayout_14")
        self.horizontalLayout_14.setContentsMargins(6, 6, 6, 6)
        self.publish_current_scene_button = QPushButton(self.groupBox3)
        self.publish_current_scene_button.setObjectName(u"publish_current_scene_button")
        sizePolicy7 = QSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        sizePolicy7.setHorizontalStretch(0)
        sizePolicy7.setVerticalStretch(0)
        sizePolicy7.setHeightForWidth(self.publish_current_scene_button.sizePolicy().hasHeightForWidth())
        self.publish_current_scene_button.setSizePolicy(sizePolicy7)
        self.publish_current_scene_button.setMinimumSize(QSize(40, 0))

        self.horizontalLayout_14.addWidget(self.publish_current_scene_button)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_14.addItem(self.horizontalSpacer_2)

        self.export_scene_geometry_text_button = QPushButton(self.groupBox3)
        self.export_scene_geometry_text_button.setObjectName(u"export_scene_geometry_text_button")
        sizePolicy7.setHeightForWidth(self.export_scene_geometry_text_button.sizePolicy().hasHeightForWidth())
        self.export_scene_geometry_text_button.setSizePolicy(sizePolicy7)
        self.export_scene_geometry_text_button.setMinimumSize(QSize(40, 0))

        self.horizontalLayout_14.addWidget(self.export_scene_geometry_text_button)

        self.import_scene_geometry_text_button = QPushButton(self.groupBox3)
        self.import_scene_geometry_text_button.setObjectName(u"import_scene_geometry_text_button")
        sizePolicy7.setHeightForWidth(self.import_scene_geometry_text_button.sizePolicy().hasHeightForWidth())
        self.import_scene_geometry_text_button.setSizePolicy(sizePolicy7)
        self.import_scene_geometry_text_button.setMinimumSize(QSize(40, 0))

        self.horizontalLayout_14.addWidget(self.import_scene_geometry_text_button)


        self.verticalLayout_20.addWidget(self.groupBox3)


        self.gridLayout_4.addLayout(self.verticalLayout_20)

        self.tabWidget.addTab(self.scene_collision_objects, "")
        self.stored_plans = QWidget()
        self.stored_plans.setObjectName(u"stored_plans")
        self.gridLayout_3 = QGridLayout(self.stored_plans)
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.groupBox_3 = QGroupBox(self.stored_plans)
        self.groupBox_3.setObjectName(u"groupBox_3")
        self.verticalLayout_7 = QVBoxLayout(self.groupBox_3)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.planning_scene_tree = QTreeWidget(self.groupBox_3)
        self.planning_scene_tree.setObjectName(u"planning_scene_tree")
        self.planning_scene_tree.viewport().setProperty("cursor", QCursor(Qt.PointingHandCursor))
        self.planning_scene_tree.setEditTriggers(QAbstractItemView.DoubleClicked|QAbstractItemView.EditKeyPressed)
        self.planning_scene_tree.setSelectionMode(QAbstractItemView.SingleSelection)
        self.planning_scene_tree.setHeaderHidden(True)
        self.planning_scene_tree.setExpandsOnDoubleClick(True)
        self.planning_scene_tree.setColumnCount(1)

        self.verticalLayout_7.addWidget(self.planning_scene_tree)


        self.verticalLayout.addWidget(self.groupBox_3)


        self.gridLayout_3.addLayout(self.verticalLayout, 1, 0, 1, 1)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.groupBox = QGroupBox(self.stored_plans)
        self.groupBox.setObjectName(u"groupBox")
        self.gridLayout_5 = QGridLayout(self.groupBox)
        self.gridLayout_5.setObjectName(u"gridLayout_5")
        self.load_scene_button = QPushButton(self.groupBox)
        self.load_scene_button.setObjectName(u"load_scene_button")
        self.load_scene_button.setEnabled(False)

        self.gridLayout_5.addWidget(self.load_scene_button, 0, 0, 1, 1)

        self.load_query_button = QPushButton(self.groupBox)
        self.load_query_button.setObjectName(u"load_query_button")
        self.load_query_button.setEnabled(False)

        self.gridLayout_5.addWidget(self.load_query_button, 0, 1, 1, 1)

        self.delete_scene_button = QPushButton(self.groupBox)
        self.delete_scene_button.setObjectName(u"delete_scene_button")
        self.delete_scene_button.setEnabled(False)

        self.gridLayout_5.addWidget(self.delete_scene_button, 1, 0, 1, 1)

        self.delete_query_button = QPushButton(self.groupBox)
        self.delete_query_button.setObjectName(u"delete_query_button")
        self.delete_query_button.setEnabled(False)

        self.gridLayout_5.addWidget(self.delete_query_button, 1, 1, 1, 1)


        self.verticalLayout_2.addWidget(self.groupBox)

        self.groupBox_2 = QGroupBox(self.stored_plans)
        self.groupBox_2.setObjectName(u"groupBox_2")
        self.gridLayout_2 = QGridLayout(self.groupBox_2)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.save_scene_button = QPushButton(self.groupBox_2)
        self.save_scene_button.setObjectName(u"save_scene_button")
        self.save_scene_button.setEnabled(False)

        self.gridLayout_2.addWidget(self.save_scene_button, 0, 1, 1, 1)

        self.save_query_button = QPushButton(self.groupBox_2)
        self.save_query_button.setObjectName(u"save_query_button")
        self.save_query_button.setEnabled(False)

        self.gridLayout_2.addWidget(self.save_query_button, 0, 0, 1, 1)

        self.verticalSpacer_7 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.gridLayout_2.addItem(self.verticalSpacer_7, 1, 0, 1, 2)


        self.verticalLayout_2.addWidget(self.groupBox_2)


        self.gridLayout_3.addLayout(self.verticalLayout_2, 1, 1, 1, 1)

        self.tabWidget.addTab(self.stored_plans, "")
        self.stored_states = QWidget()
        self.stored_states.setObjectName(u"stored_states")
        self.horizontalLayout_2 = QHBoxLayout(self.stored_states)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.list_states = QListWidget(self.stored_states)
        self.list_states.setObjectName(u"list_states")

        self.horizontalLayout_2.addWidget(self.list_states)

        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.groupBox_15 = QGroupBox(self.stored_states)
        self.groupBox_15.setObjectName(u"groupBox_15")
        self.gridLayout_17 = QGridLayout(self.groupBox_15)
        self.gridLayout_17.setObjectName(u"gridLayout_17")
        self.load_state_button = QPushButton(self.groupBox_15)
        self.load_state_button.setObjectName(u"load_state_button")

        self.gridLayout_17.addWidget(self.load_state_button, 0, 0, 1, 1)

        self.clear_states_button = QPushButton(self.groupBox_15)
        self.clear_states_button.setObjectName(u"clear_states_button")

        self.gridLayout_17.addWidget(self.clear_states_button, 0, 1, 1, 1)


        self.verticalLayout_4.addWidget(self.groupBox_15)

        self.groupBox_16 = QGroupBox(self.stored_states)
        self.groupBox_16.setObjectName(u"groupBox_16")
        self.gridLayout_15 = QGridLayout(self.groupBox_16)
        self.gridLayout_15.setObjectName(u"gridLayout_15")
        self.set_as_start_state_button = QPushButton(self.groupBox_16)
        self.set_as_start_state_button.setObjectName(u"set_as_start_state_button")

        self.gridLayout_15.addWidget(self.set_as_start_state_button, 0, 0, 1, 1)

        self.set_as_goal_state_button = QPushButton(self.groupBox_16)
        self.set_as_goal_state_button.setObjectName(u"set_as_goal_state_button")

        self.gridLayout_15.addWidget(self.set_as_goal_state_button, 0, 1, 1, 1)

        self.remove_state_button = QPushButton(self.groupBox_16)
        self.remove_state_button.setObjectName(u"remove_state_button")

        self.gridLayout_15.addWidget(self.remove_state_button, 1, 0, 1, 1)


        self.verticalLayout_4.addWidget(self.groupBox_16)

        self.groupBox_17 = QGroupBox(self.stored_states)
        self.groupBox_17.setObjectName(u"groupBox_17")
        self.gridLayout_16 = QGridLayout(self.groupBox_17)
        self.gridLayout_16.setObjectName(u"gridLayout_16")
        self.save_start_state_button = QPushButton(self.groupBox_17)
        self.save_start_state_button.setObjectName(u"save_start_state_button")

        self.gridLayout_16.addWidget(self.save_start_state_button, 0, 0, 1, 1)

        self.save_goal_state_button = QPushButton(self.groupBox_17)
        self.save_goal_state_button.setObjectName(u"save_goal_state_button")

        self.gridLayout_16.addWidget(self.save_goal_state_button, 0, 1, 1, 1)


        self.verticalLayout_4.addWidget(self.groupBox_17)

        self.verticalSpacer_8 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_4.addItem(self.verticalSpacer_8)


        self.horizontalLayout_2.addLayout(self.verticalLayout_4)

        self.tabWidget.addTab(self.stored_states, "")
        self.status = QWidget()
        self.status.setObjectName(u"status")
        self.gridLayout_11 = QGridLayout(self.status)
        self.gridLayout_11.setObjectName(u"gridLayout_11")
        self.status_text = QTextEdit(self.status)
        self.status_text.setObjectName(u"status_text")
        self.status_text.setReadOnly(True)

        self.gridLayout_11.addWidget(self.status_text, 0, 0, 1, 1)

        self.tabWidget.addTab(self.status, "")
        self.manipulation = QWidget()
        self.manipulation.setObjectName(u"manipulation")
        self.gridLayout_13 = QGridLayout(self.manipulation)
        self.gridLayout_13.setObjectName(u"gridLayout_13")
        self.groupBox_13 = QGroupBox(self.manipulation)
        self.groupBox_13.setObjectName(u"groupBox_13")
        self.verticalLayout_8 = QVBoxLayout(self.groupBox_13)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.detected_objects_list = QListWidget(self.groupBox_13)
        self.detected_objects_list.setObjectName(u"detected_objects_list")

        self.verticalLayout_8.addWidget(self.detected_objects_list)

        self.detect_objects_button = QPushButton(self.groupBox_13)
        self.detect_objects_button.setObjectName(u"detect_objects_button")

        self.verticalLayout_8.addWidget(self.detect_objects_button)


        self.gridLayout_13.addWidget(self.groupBox_13, 0, 0, 2, 1)

        self.groupBox_14 = QGroupBox(self.manipulation)
        self.groupBox_14.setObjectName(u"groupBox_14")
        self.gridLayout_12 = QGridLayout(self.groupBox_14)
        self.gridLayout_12.setObjectName(u"gridLayout_12")
        self.place_button = QPushButton(self.groupBox_14)
        self.place_button.setObjectName(u"place_button")
        self.place_button.setEnabled(False)

        self.gridLayout_12.addWidget(self.place_button, 1, 1, 1, 1)

        self.verticalSpacer_9 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.gridLayout_12.addItem(self.verticalSpacer_9, 2, 1, 1, 1)

        self.pick_button = QPushButton(self.groupBox_14)
        self.pick_button.setObjectName(u"pick_button")
        self.pick_button.setEnabled(False)

        self.gridLayout_12.addWidget(self.pick_button, 0, 1, 1, 1)

        self.support_surfaces_list = QListWidget(self.groupBox_14)
        self.support_surfaces_list.setObjectName(u"support_surfaces_list")

        self.gridLayout_12.addWidget(self.support_surfaces_list, 0, 0, 3, 1)


        self.gridLayout_13.addWidget(self.groupBox_14, 0, 1, 1, 1)

        self.groupBox_7 = QGroupBox(self.manipulation)
        self.groupBox_7.setObjectName(u"groupBox_7")
        self.gridLayout_14 = QGridLayout(self.groupBox_7)
        self.gridLayout_14.setObjectName(u"gridLayout_14")
        self.roi_center_y = QDoubleSpinBox(self.groupBox_7)
        self.roi_center_y.setObjectName(u"roi_center_y")
        sizePolicy2.setHeightForWidth(self.roi_center_y.sizePolicy().hasHeightForWidth())
        self.roi_center_y.setSizePolicy(sizePolicy2)
        self.roi_center_y.setMinimumSize(QSize(40, 0))
        self.roi_center_y.setMinimum(-99.000000000000000)
        self.roi_center_y.setSingleStep(0.010000000000000)

        self.gridLayout_14.addWidget(self.roi_center_y, 0, 2, 1, 1)

        self.roi_size_x = QDoubleSpinBox(self.groupBox_7)
        self.roi_size_x.setObjectName(u"roi_size_x")
        sizePolicy2.setHeightForWidth(self.roi_size_x.sizePolicy().hasHeightForWidth())
        self.roi_size_x.setSizePolicy(sizePolicy2)
        self.roi_size_x.setMinimumSize(QSize(40, 0))
        self.roi_size_x.setSingleStep(0.010000000000000)
        self.roi_size_x.setValue(4.000000000000000)

        self.gridLayout_14.addWidget(self.roi_size_x, 1, 1, 1, 1)

        self.roi_size_z = QDoubleSpinBox(self.groupBox_7)
        self.roi_size_z.setObjectName(u"roi_size_z")
        sizePolicy2.setHeightForWidth(self.roi_size_z.sizePolicy().hasHeightForWidth())
        self.roi_size_z.setSizePolicy(sizePolicy2)
        self.roi_size_z.setMinimumSize(QSize(40, 0))
        self.roi_size_z.setSingleStep(0.010000000000000)
        self.roi_size_z.setValue(1.200000000000000)

        self.gridLayout_14.addWidget(self.roi_size_z, 1, 3, 1, 1)

        self.roi_size_y = QDoubleSpinBox(self.groupBox_7)
        self.roi_size_y.setObjectName(u"roi_size_y")
        sizePolicy2.setHeightForWidth(self.roi_size_y.sizePolicy().hasHeightForWidth())
        self.roi_size_y.setSizePolicy(sizePolicy2)
        self.roi_size_y.setMinimumSize(QSize(40, 0))
        self.roi_size_y.setSingleStep(0.010000000000000)
        self.roi_size_y.setValue(1.500000000000000)

        self.gridLayout_14.addWidget(self.roi_size_y, 1, 2, 1, 1)

        self.label_14 = QLabel(self.groupBox_7)
        self.label_14.setObjectName(u"label_14")

        self.gridLayout_14.addWidget(self.label_14, 1, 0, 1, 1)

        self.roi_center_x = QDoubleSpinBox(self.groupBox_7)
        self.roi_center_x.setObjectName(u"roi_center_x")
        sizePolicy2.setHeightForWidth(self.roi_center_x.sizePolicy().hasHeightForWidth())
        self.roi_center_x.setSizePolicy(sizePolicy2)
        self.roi_center_x.setMinimumSize(QSize(40, 0))
        self.roi_center_x.setMinimum(-99.000000000000000)
        self.roi_center_x.setSingleStep(0.010000000000000)

        self.gridLayout_14.addWidget(self.roi_center_x, 0, 1, 1, 1)

        self.label_12 = QLabel(self.groupBox_7)
        self.label_12.setObjectName(u"label_12")

        self.gridLayout_14.addWidget(self.label_12, 0, 0, 1, 1)

        self.roi_center_z = QDoubleSpinBox(self.groupBox_7)
        self.roi_center_z.setObjectName(u"roi_center_z")
        sizePolicy2.setHeightForWidth(self.roi_center_z.sizePolicy().hasHeightForWidth())
        self.roi_center_z.setSizePolicy(sizePolicy2)
        self.roi_center_z.setMinimumSize(QSize(40, 0))
        self.roi_center_z.setMinimum(-99.000000000000000)
        self.roi_center_z.setSingleStep(0.010000000000000)
        self.roi_center_z.setValue(0.600000000000000)

        self.gridLayout_14.addWidget(self.roi_center_z, 0, 3, 1, 1)


        self.gridLayout_13.addWidget(self.groupBox_7, 1, 1, 1, 1)

        self.tabWidget.addTab(self.manipulation, "")

        self.verticalLayout_5.addWidget(self.tabWidget)

        self.background_job_progress = QProgressBar(MotionPlanningUI)
        self.background_job_progress.setObjectName(u"background_job_progress")
        sizePolicy8 = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        sizePolicy8.setHorizontalStretch(0)
        sizePolicy8.setVerticalStretch(0)
        sizePolicy8.setHeightForWidth(self.background_job_progress.sizePolicy().hasHeightForWidth())
        self.background_job_progress.setSizePolicy(sizePolicy8)
        self.background_job_progress.setMaximumSize(QSize(16777215, 5))
        self.background_job_progress.setLayoutDirection(Qt.LeftToRight)
        self.background_job_progress.setAutoFillBackground(False)
        self.background_job_progress.setStyleSheet(u"")
        self.background_job_progress.setMinimum(0)
        self.background_job_progress.setMaximum(1)
        self.background_job_progress.setValue(0)

        self.verticalLayout_5.addWidget(self.background_job_progress)

        QWidget.setTabOrder(self.planning_algorithm_combo_box, self.planner_param_treeview)
        QWidget.setTabOrder(self.planner_param_treeview, self.database_host)
        QWidget.setTabOrder(self.database_host, self.database_port)
        QWidget.setTabOrder(self.database_port, self.reset_db_button)
        QWidget.setTabOrder(self.reset_db_button, self.database_connect_button)
        QWidget.setTabOrder(self.database_connect_button, self.wcenter_x)
        QWidget.setTabOrder(self.wcenter_x, self.wcenter_y)
        QWidget.setTabOrder(self.wcenter_y, self.wcenter_z)
        QWidget.setTabOrder(self.wcenter_z, self.wsize_x)
        QWidget.setTabOrder(self.wsize_x, self.wsize_y)
        QWidget.setTabOrder(self.wsize_y, self.wsize_z)
        QWidget.setTabOrder(self.wsize_z, self.plan_button)
        QWidget.setTabOrder(self.plan_button, self.execute_button)
        QWidget.setTabOrder(self.execute_button, self.plan_and_execute_button)
        QWidget.setTabOrder(self.plan_and_execute_button, self.stop_button)
        QWidget.setTabOrder(self.stop_button, self.clear_octomap_button)
        QWidget.setTabOrder(self.clear_octomap_button, self.planning_group_combo_box)
        QWidget.setTabOrder(self.planning_group_combo_box, self.start_state_combo_box)
        QWidget.setTabOrder(self.start_state_combo_box, self.goal_state_combo_box)
        QWidget.setTabOrder(self.goal_state_combo_box, self.path_constraints_combo_box)
        QWidget.setTabOrder(self.path_constraints_combo_box, self.planning_time)
        QWidget.setTabOrder(self.planning_time, self.planning_attempts)
        QWidget.setTabOrder(self.planning_attempts, self.velocity_scaling_factor)
        QWidget.setTabOrder(self.velocity_scaling_factor, self.acceleration_scaling_factor)
        QWidget.setTabOrder(self.acceleration_scaling_factor, self.use_cartesian_path)
        QWidget.setTabOrder(self.use_cartesian_path, self.collision_aware_ik)
        QWidget.setTabOrder(self.collision_aware_ik, self.approximate_ik)
        QWidget.setTabOrder(self.approximate_ik, self.allow_external_program)
        QWidget.setTabOrder(self.allow_external_program, self.allow_replanning)
        QWidget.setTabOrder(self.allow_replanning, self.allow_looking)
        QWidget.setTabOrder(self.allow_looking, self.detected_objects_list)
        QWidget.setTabOrder(self.detected_objects_list, self.detect_objects_button)
        QWidget.setTabOrder(self.detect_objects_button, self.support_surfaces_list)
        QWidget.setTabOrder(self.support_surfaces_list, self.pick_button)
        QWidget.setTabOrder(self.pick_button, self.place_button)
        QWidget.setTabOrder(self.place_button, self.roi_center_x)
        QWidget.setTabOrder(self.roi_center_x, self.roi_center_y)
        QWidget.setTabOrder(self.roi_center_y, self.roi_center_z)
        QWidget.setTabOrder(self.roi_center_z, self.roi_size_x)
        QWidget.setTabOrder(self.roi_size_x, self.roi_size_y)
        QWidget.setTabOrder(self.roi_size_y, self.roi_size_z)
        QWidget.setTabOrder(self.roi_size_z, self.collision_objects_list)
        QWidget.setTabOrder(self.collision_objects_list, self.shape_size_x_spin_box)
        QWidget.setTabOrder(self.shape_size_x_spin_box, self.shape_size_z_spin_box)
        QWidget.setTabOrder(self.shape_size_z_spin_box, self.shape_size_y_spin_box)
        QWidget.setTabOrder(self.shape_size_y_spin_box, self.shapes_combo_box)
        QWidget.setTabOrder(self.shapes_combo_box, self.add_object_button)
        QWidget.setTabOrder(self.add_object_button, self.remove_object_button)
        QWidget.setTabOrder(self.remove_object_button, self.clear_scene_button)
        QWidget.setTabOrder(self.clear_scene_button, self.object_x)
        QWidget.setTabOrder(self.object_x, self.object_y)
        QWidget.setTabOrder(self.object_y, self.object_z)
        QWidget.setTabOrder(self.object_z, self.object_rx)
        QWidget.setTabOrder(self.object_rx, self.object_ry)
        QWidget.setTabOrder(self.object_ry, self.object_rz)
        QWidget.setTabOrder(self.object_rz, self.scene_scale)
        QWidget.setTabOrder(self.scene_scale, self.publish_current_scene_button)
        QWidget.setTabOrder(self.publish_current_scene_button, self.export_scene_geometry_text_button)
        QWidget.setTabOrder(self.export_scene_geometry_text_button, self.import_scene_geometry_text_button)
        QWidget.setTabOrder(self.import_scene_geometry_text_button, self.planning_scene_tree)
        QWidget.setTabOrder(self.planning_scene_tree, self.load_scene_button)
        QWidget.setTabOrder(self.load_scene_button, self.load_query_button)
        QWidget.setTabOrder(self.load_query_button, self.delete_scene_button)
        QWidget.setTabOrder(self.delete_scene_button, self.delete_query_button)
        QWidget.setTabOrder(self.delete_query_button, self.save_query_button)
        QWidget.setTabOrder(self.save_query_button, self.save_scene_button)
        QWidget.setTabOrder(self.save_scene_button, self.list_states)
        QWidget.setTabOrder(self.list_states, self.load_state_button)
        QWidget.setTabOrder(self.load_state_button, self.clear_states_button)
        QWidget.setTabOrder(self.clear_states_button, self.set_as_start_state_button)
        QWidget.setTabOrder(self.set_as_start_state_button, self.set_as_goal_state_button)
        QWidget.setTabOrder(self.set_as_goal_state_button, self.remove_state_button)
        QWidget.setTabOrder(self.remove_state_button, self.save_start_state_button)
        QWidget.setTabOrder(self.save_start_state_button, self.save_goal_state_button)
        QWidget.setTabOrder(self.save_goal_state_button, self.status_text)
        QWidget.setTabOrder(self.status_text, self.tabWidget)

        self.retranslateUi(MotionPlanningUI)

        self.tabWidget.setCurrentIndex(1)
        self.reset_db_button.setDefault(False)
        self.database_connect_button.setDefault(False)


        QMetaObject.connectSlotsByName(MotionPlanningUI)
    # setupUi

    def retranslateUi(self, MotionPlanningUI):
        MotionPlanningUI.setWindowTitle(QCoreApplication.translate("MotionPlanningUI", u"MoveIt Planning Frame", None))
        self.groupBox1.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Planning Library", None))
        self.library_label.setText(QCoreApplication.translate("MotionPlanningUI", u"Planning Library Name", None))
        self.label_15.setText(QCoreApplication.translate("MotionPlanningUI", u"Planner Parameters", None))
        self.groupBox2.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Warehouse", None))
        self.label_2.setText(QCoreApplication.translate("MotionPlanningUI", u"Host:", None))
        self.database_host.setText("")
        self.label_3.setText(QCoreApplication.translate("MotionPlanningUI", u"Port:", None))
        self.reset_db_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Reset database ...", None))
        self.database_connect_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Connect", None))
        self.groupBox_6.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Workspace", None))
        self.label_21.setText(QCoreApplication.translate("MotionPlanningUI", u"Center (XYZ):", None))
        self.label_22.setText(QCoreApplication.translate("MotionPlanningUI", u"Size (XYZ):", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.context), QCoreApplication.translate("MotionPlanningUI", u"Context", None))
        self.groupBox_9.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Commands", None))
        self.plan_button.setText(QCoreApplication.translate("MotionPlanningUI", u"&Plan", None))
        self.execute_button.setText(QCoreApplication.translate("MotionPlanningUI", u"&Execute", None))
        self.plan_and_execute_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Plan && E&xecute", None))
#if QT_CONFIG(tooltip)
        self.stop_button.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Stop execution by sending a stop command to the move_group.", None))
#endif // QT_CONFIG(tooltip)
        self.stop_button.setText(QCoreApplication.translate("MotionPlanningUI", u"&Stop", None))
        self.result_label.setText("")
        self.clear_octomap_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Clear octomap", None))
        self.groupBox_11.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Query", None))
        self.label_planning_group.setText(QCoreApplication.translate("MotionPlanningUI", u"Planning Group:", None))
        self.label_start_state.setText(QCoreApplication.translate("MotionPlanningUI", u"Start State:", None))
        self.label_goal_state.setText(QCoreApplication.translate("MotionPlanningUI", u"Goal State:", None))
        self.groupBox_12.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Path Constraints", None))
#if QT_CONFIG(tooltip)
        self.path_constraints_combo_box.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Select path constraints from the database. ", None))
#endif // QT_CONFIG(tooltip)
        self.groupBox_10.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Options", None))
#if QT_CONFIG(tooltip)
        self.label_9.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Total time allowed for planning.  Planning stops if time or number of attempts is exceeded, or goal is reached.", None))
#endif // QT_CONFIG(tooltip)
        self.label_9.setText(QCoreApplication.translate("MotionPlanningUI", u"Planning Time (s):", None))
#if QT_CONFIG(tooltip)
        self.planning_time.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Total time allowed for planning.  Planning stops if time or number of attempts is exceeded, or goal is reached.", None))
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        self.label_10.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Allowed number of planning attempts. Planning stops if time or number of attempts is exceeded, or goal is reached.", None))
#endif // QT_CONFIG(tooltip)
        self.label_10.setText(QCoreApplication.translate("MotionPlanningUI", u"Planning Attempts:", None))
#if QT_CONFIG(tooltip)
        self.planning_attempts.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Allowed number of planning attempts. Planning stops if time or number of attempts is exceeded, or goal is reached.", None))
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        self.label_11.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Factor (between 0 and 1) to scale down maximum joint velocities", None))
#endif // QT_CONFIG(tooltip)
        self.label_11.setText(QCoreApplication.translate("MotionPlanningUI", u"Velocity Scaling:", None))
#if QT_CONFIG(tooltip)
        self.velocity_scaling_factor.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Factor (between 0 and 1) to scale down maximum joint velocities", None))
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        self.label_20.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Factor (between 0 and 1) to scale down maximum joint accelerations", None))
#endif // QT_CONFIG(tooltip)
        self.label_20.setText(QCoreApplication.translate("MotionPlanningUI", u"Accel. Scaling:", None))
#if QT_CONFIG(tooltip)
        self.acceleration_scaling_factor.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Factor (between 0 and 1) to scale down maximum joint accelerations", None))
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        self.use_cartesian_path.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Check this to generate a linear path in Cartesian (3D) space.\n"
"\n"
" This does not plan around obstacles. ", None))
#endif // QT_CONFIG(tooltip)
        self.use_cartesian_path.setText(QCoreApplication.translate("MotionPlanningUI", u"Use Cartesian Path", None))
#if QT_CONFIG(tooltip)
        self.collision_aware_ik.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"If checked, the IK solver tries to avoid collisions when searching for a start/end state set via the interactive marker.\n"
"\n"
"This is usually achieved by random seeding, which can flip the robot configuration (e.g. elbow up/down). Note that motion planning is always collision-aware, regardless of this checkbox.", None))
#endif // QT_CONFIG(tooltip)
        self.collision_aware_ik.setText(QCoreApplication.translate("MotionPlanningUI", u"Collision-aware IK", None))
#if QT_CONFIG(tooltip)
        self.approximate_ik.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Allow approximate IK solutions, which deviate from the target pose. Useful when an exact solution cannot be found, e.g. for underactuated arms or when trying to reach an unreachable target.", None))
#endif // QT_CONFIG(tooltip)
        self.approximate_ik.setText(QCoreApplication.translate("MotionPlanningUI", u"Approx IK Solutions", None))
#if QT_CONFIG(tooltip)
        self.allow_external_program.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Allow other nodes to control start and end poses as well as planning and execution, using ROS topics (/rviz/moveit/ ).", None))
#endif // QT_CONFIG(tooltip)
        self.allow_external_program.setText(QCoreApplication.translate("MotionPlanningUI", u"External Comm.", None))
#if QT_CONFIG(tooltip)
        self.allow_replanning.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"EXPERIMENTAL: Trigger replanning if new collisions are detected during plan execution. Only works for the \"Plan & Execute\" button. ", None))
#endif // QT_CONFIG(tooltip)
        self.allow_replanning.setText(QCoreApplication.translate("MotionPlanningUI", u"Replanning", None))
#if QT_CONFIG(tooltip)
        self.allow_looking.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"EXPERIMENTAL: Allows the robot to reposition its sensor to look around to resolve apparent collisions. A sensor needs to be set up and the new measurements have to affect the collision scene, otherwise it has no effect. See tutorial.", None))
#endif // QT_CONFIG(tooltip)
        self.allow_looking.setText(QCoreApplication.translate("MotionPlanningUI", u"Sensor Positioning", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.planning), QCoreApplication.translate("MotionPlanningUI", u"Planning", None))
        self.groupBox_8.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Current Scene Objects", None))
#if QT_CONFIG(tooltip)
        self.collision_objects_list.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Press Ctrl+C to duplicate an object", None))
#endif // QT_CONFIG(tooltip)
        self.groupBox_4.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Add/Remove scene object(s)", None))
#if QT_CONFIG(tooltip)
        self.shape_size_x_spin_box.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"x dimension", None))
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        self.shape_size_y_spin_box.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"y dimension", None))
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        self.shape_size_z_spin_box.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"z dimension", None))
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        self.add_object_button.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Add object at scene's origin", None))
#endif // QT_CONFIG(tooltip)
        self.add_object_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Add", None))
#if QT_CONFIG(tooltip)
        self.remove_object_button.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Remove selected object", None))
#endif // QT_CONFIG(tooltip)
        self.remove_object_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Del", None))
#if QT_CONFIG(tooltip)
        self.clear_scene_button.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Clear planning scene", None))
#endif // QT_CONFIG(tooltip)
        self.clear_scene_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Clr", None))
#if QT_CONFIG(tooltip)
        self.pose_scale_group_box.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Change the pose and scale of the selected object", None))
#endif // QT_CONFIG(tooltip)
        self.pose_scale_group_box.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Change object pose/scale", None))
        self.label_4.setText(QCoreApplication.translate("MotionPlanningUI", u"Position: ", None))
        self.label_13.setText(QCoreApplication.translate("MotionPlanningUI", u"Rotation:", None))
        self.label.setText(QCoreApplication.translate("MotionPlanningUI", u"Scale:", None))
        self.label_17.setText(QCoreApplication.translate("MotionPlanningUI", u"0%", None))
        self.label_18.setText(QCoreApplication.translate("MotionPlanningUI", u"200%", None))
#if QT_CONFIG(tooltip)
        self.groupBox_5.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Describes the selected object's pose, geometry and subframes.", None))
#endif // QT_CONFIG(tooltip)
        self.groupBox_5.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Object status", None))
        self.object_status.setText(QCoreApplication.translate("MotionPlanningUI", u"Status", None))
        self.groupBox3.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Scene Geometry", None))
        self.publish_current_scene_button.setText(QCoreApplication.translate("MotionPlanningUI", u"&Publish", None))
#if QT_CONFIG(tooltip)
        self.export_scene_geometry_text_button.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Export scene geometry to a .scene file (only world object information is preserved)", None))
#endif // QT_CONFIG(tooltip)
        self.export_scene_geometry_text_button.setText(QCoreApplication.translate("MotionPlanningUI", u"&Export", None))
#if QT_CONFIG(tooltip)
        self.import_scene_geometry_text_button.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Import scene geometry from a .scene file (only world object information is preserved)", None))
#endif // QT_CONFIG(tooltip)
        self.import_scene_geometry_text_button.setText(QCoreApplication.translate("MotionPlanningUI", u"&Import", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.scene_collision_objects), QCoreApplication.translate("MotionPlanningUI", u"Scene Objects", None))
        self.groupBox_3.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Planning Scenes and Queries", None))
        ___qtreewidgetitem = self.planning_scene_tree.headerItem()
        ___qtreewidgetitem.setText(0, QCoreApplication.translate("MotionPlanningUI", u"1", None));
        self.groupBox.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Saved Scenes", None))
        self.load_scene_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Load &Scene", None))
        self.load_query_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Load &Query", None))
        self.delete_scene_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Delete Scene", None))
        self.delete_query_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Delete Query", None))
        self.groupBox_2.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Current Scene", None))
        self.save_scene_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Save Scene", None))
        self.save_query_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Save Query", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.stored_plans), QCoreApplication.translate("MotionPlanningUI", u"Stored Scenes", None))
        self.groupBox_15.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Stored Robot States", None))
#if QT_CONFIG(tooltip)
        self.load_state_button.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Load a robot state", None))
#endif // QT_CONFIG(tooltip)
        self.load_state_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Load", None))
#if QT_CONFIG(tooltip)
        self.clear_states_button.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Clear robot states from list", None))
#endif // QT_CONFIG(tooltip)
        self.clear_states_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Clear", None))
        self.groupBox_16.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Selected Robot State", None))
        self.set_as_start_state_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Set as Start", None))
        self.set_as_goal_state_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Set as Goal", None))
        self.remove_state_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Remove", None))
        self.groupBox_17.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Current Robot State", None))
#if QT_CONFIG(tooltip)
        self.save_start_state_button.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Save the current start state", None))
#endif // QT_CONFIG(tooltip)
        self.save_start_state_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Save Start", None))
#if QT_CONFIG(tooltip)
        self.save_goal_state_button.setToolTip(QCoreApplication.translate("MotionPlanningUI", u"Save the current goal state", None))
#endif // QT_CONFIG(tooltip)
        self.save_goal_state_button.setText(QCoreApplication.translate("MotionPlanningUI", u"Save Goal", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.stored_states), QCoreApplication.translate("MotionPlanningUI", u"Stored States", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.status), QCoreApplication.translate("MotionPlanningUI", u"Status", None))
        self.groupBox_13.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Detected Objects", None))
        self.detect_objects_button.setText(QCoreApplication.translate("MotionPlanningUI", u"&Detect", None))
        self.groupBox_14.setTitle(QCoreApplication.translate("MotionPlanningUI", u"Support Surfaces", None))
        self.place_button.setText(QCoreApplication.translate("MotionPlanningUI", u"P&lace", None))
        self.pick_button.setText(QCoreApplication.translate("MotionPlanningUI", u"&Pick", None))
        self.groupBox_7.setTitle(QCoreApplication.translate("MotionPlanningUI", u"ROI", None))
        self.label_14.setText(QCoreApplication.translate("MotionPlanningUI", u"Size (m): ", None))
        self.label_12.setText(QCoreApplication.translate("MotionPlanningUI", u"Center (m): ", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.manipulation), QCoreApplication.translate("MotionPlanningUI", u"Manipulation", None))
#if QT_CONFIG(whatsthis)
        self.background_job_progress.setWhatsThis(QCoreApplication.translate("MotionPlanningUI", u"<html><head/><body><p>Progress of background jobs</p></body></html>", None))
#endif // QT_CONFIG(whatsthis)
        self.background_job_progress.setFormat("")
    # retranslateUi

