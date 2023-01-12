# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'motion_planning_rviz_plugin_frame_joints.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_MotionPlanningFrameJointsUI(object):
    def setupUi(self, MotionPlanningFrameJointsUI):
        if not MotionPlanningFrameJointsUI.objectName():
            MotionPlanningFrameJointsUI.setObjectName(u"MotionPlanningFrameJointsUI")
        MotionPlanningFrameJointsUI.resize(400, 300)
        self.horizontalLayout = QHBoxLayout(MotionPlanningFrameJointsUI)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.joints_layout_ = QVBoxLayout()
        self.joints_layout_.setObjectName(u"joints_layout_")
        self.joints_view_label_ = QLabel(MotionPlanningFrameJointsUI)
        self.joints_view_label_.setObjectName(u"joints_view_label_")

        self.joints_layout_.addWidget(self.joints_view_label_)

        self.joints_view_ = QTreeView(MotionPlanningFrameJointsUI)
        self.joints_view_.setObjectName(u"joints_view_")
        self.joints_view_.setEditTriggers(QAbstractItemView.EditKeyPressed)
        self.joints_view_.setSelectionMode(QAbstractItemView.NoSelection)
        self.joints_view_.setRootIsDecorated(False)
        self.joints_view_.setItemsExpandable(False)
        self.joints_view_.setExpandsOnDoubleClick(False)

        self.joints_layout_.addWidget(self.joints_view_)

        self.nullspace_label = QLabel(MotionPlanningFrameJointsUI)
        self.nullspace_label.setObjectName(u"nullspace_label")

        self.joints_layout_.addWidget(self.nullspace_label)

        self.nullspace_layout_ = QVBoxLayout()
        self.nullspace_layout_.setObjectName(u"nullspace_layout_")
        self.dummy_ns_slider_ = QSlider(MotionPlanningFrameJointsUI)
        self.dummy_ns_slider_.setObjectName(u"dummy_ns_slider_")
        self.dummy_ns_slider_.setEnabled(False)
        self.dummy_ns_slider_.setMinimum(-1)
        self.dummy_ns_slider_.setMaximum(1)
        self.dummy_ns_slider_.setOrientation(Qt.Horizontal)

        self.nullspace_layout_.addWidget(self.dummy_ns_slider_)


        self.joints_layout_.addLayout(self.nullspace_layout_)


        self.horizontalLayout.addLayout(self.joints_layout_)


        self.retranslateUi(MotionPlanningFrameJointsUI)

        QMetaObject.connectSlotsByName(MotionPlanningFrameJointsUI)
    # setupUi

    def retranslateUi(self, MotionPlanningFrameJointsUI):
        MotionPlanningFrameJointsUI.setWindowTitle(QCoreApplication.translate("MotionPlanningFrameJointsUI", u"Form", None))
        self.joints_view_label_.setText(QCoreApplication.translate("MotionPlanningFrameJointsUI", u"Group joints:", None))
#if QT_CONFIG(tooltip)
        self.nullspace_label.setToolTip(QCoreApplication.translate("MotionPlanningFrameJointsUI", u"The sliders below allow for jogging the nullspace of the current configuration,\n"
"i.e. trigger joint motions that don't affect the end-effector pose.\n"
"\n"
"Typically, redundant arms (with 7+ joints) offer such a nullspace.\n"
"However, also singular configurations provide a nullspace.\n"
"\n"
"Each basis vector of the (linear) nullspace is represented by a separate slider.", None))
#endif // QT_CONFIG(tooltip)
        self.nullspace_label.setText(QCoreApplication.translate("MotionPlanningFrameJointsUI", u"Nullspace exploration:", None))
#if QT_CONFIG(tooltip)
        self.dummy_ns_slider_.setToolTip(QCoreApplication.translate("MotionPlanningFrameJointsUI", u"The slider will become active if the current robot configuration has a nullspace.\n"
"That's typically the case for redundant robots, i.e. 7+ joints, or singular configurations.", None))
#endif // QT_CONFIG(tooltip)
    # retranslateUi

