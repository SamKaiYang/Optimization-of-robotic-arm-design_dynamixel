# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'main.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(966, 560)
        palette = QPalette()
        brush = QBrush(QColor(255, 255, 255, 255))
        brush.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.Base, brush)
        brush1 = QBrush(QColor(238, 238, 236, 255))
        brush1.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.Window, brush1)
        palette.setBrush(QPalette.Inactive, QPalette.Base, brush)
        palette.setBrush(QPalette.Inactive, QPalette.Window, brush1)
        palette.setBrush(QPalette.Disabled, QPalette.Base, brush1)
        palette.setBrush(QPalette.Disabled, QPalette.Window, brush1)
        MainWindow.setPalette(palette)
        font = QFont()
        font.setFamily(u"Bitstream Vera Sans Mono")
        MainWindow.setFont(font)
        icon = QIcon()
        icon.addFile(u"src/modbus/modbus/picture/teco_icon.png", QSize(), QIcon.Normal, QIcon.Off)
        MainWindow.setWindowIcon(icon)
        MainWindow.setToolButtonStyle(Qt.ToolButtonIconOnly)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tabWidget.setGeometry(QRect(0, 10, 961, 501))
        font1 = QFont()
        font1.setFamily(u"Bitstream Vera Sans Mono")
        font1.setBold(True)
        font1.setWeight(75)
        self.tabWidget.setFont(font1)
        self.tabWidget.setTabShape(QTabWidget.Triangular)
        self.tab_arm = QWidget()
        self.tab_arm.setObjectName(u"tab_arm")
        self.lineEdit_ip_2 = QLineEdit(self.tab_arm)
        self.lineEdit_ip_2.setObjectName(u"lineEdit_ip_2")
        self.lineEdit_ip_2.setGeometry(QRect(20, 40, 91, 26))
        self.lineEdit_ip_3 = QLineEdit(self.tab_arm)
        self.lineEdit_ip_3.setObjectName(u"lineEdit_ip_3")
        self.lineEdit_ip_3.setGeometry(QRect(20, 80, 91, 26))
        self.tabWidget.addTab(self.tab_arm, "")
        self.tab_mission = QWidget()
        self.tab_mission.setObjectName(u"tab_mission")
        self.btn_dynamics = QPushButton(self.tab_mission)
        self.btn_dynamics.setObjectName(u"btn_dynamics")
        self.btn_dynamics.setGeometry(QRect(740, 420, 201, 33))
        palette1 = QPalette()
        palette1.setBrush(QPalette.Active, QPalette.WindowText, brush)
        brush2 = QBrush(QColor(218, 119, 0, 255))
        brush2.setStyle(Qt.SolidPattern)
        palette1.setBrush(QPalette.Active, QPalette.Button, brush2)
        palette1.setBrush(QPalette.Active, QPalette.Text, brush)
        palette1.setBrush(QPalette.Active, QPalette.ButtonText, brush)
        palette1.setBrush(QPalette.Active, QPalette.Base, brush2)
        palette1.setBrush(QPalette.Active, QPalette.Window, brush2)
        palette1.setBrush(QPalette.Inactive, QPalette.WindowText, brush)
        palette1.setBrush(QPalette.Inactive, QPalette.Button, brush2)
        palette1.setBrush(QPalette.Inactive, QPalette.Text, brush)
        palette1.setBrush(QPalette.Inactive, QPalette.ButtonText, brush)
        palette1.setBrush(QPalette.Inactive, QPalette.Base, brush2)
        palette1.setBrush(QPalette.Inactive, QPalette.Window, brush2)
        palette1.setBrush(QPalette.Disabled, QPalette.WindowText, brush)
        palette1.setBrush(QPalette.Disabled, QPalette.Button, brush2)
        palette1.setBrush(QPalette.Disabled, QPalette.Text, brush)
        palette1.setBrush(QPalette.Disabled, QPalette.ButtonText, brush)
        palette1.setBrush(QPalette.Disabled, QPalette.Base, brush2)
        palette1.setBrush(QPalette.Disabled, QPalette.Window, brush2)
        self.btn_dynamics.setPalette(palette1)
        font2 = QFont()
        font2.setFamily(u"Bitstream Vera Sans")
        font2.setPointSize(11)
        font2.setBold(True)
        font2.setWeight(75)
        self.btn_dynamics.setFont(font2)
        self.btn_dynamics.setStyleSheet(u"background-color:#da7700;color:white;border-color: black;")
        self.lineEdit_payload = QLineEdit(self.tab_mission)
        self.lineEdit_payload.setObjectName(u"lineEdit_payload")
        self.lineEdit_payload.setGeometry(QRect(20, 200, 71, 26))
        self.lineEdit_vel_0 = QLineEdit(self.tab_mission)
        self.lineEdit_vel_0.setObjectName(u"lineEdit_vel_0")
        self.lineEdit_vel_0.setGeometry(QRect(20, 280, 61, 26))
        self.label = QLabel(self.tab_mission)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 170, 67, 17))
        self.label_2 = QLabel(self.tab_mission)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(20, 240, 151, 17))
        self.label_3 = QLabel(self.tab_mission)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(20, 320, 191, 17))
        self.lineEdit_vel_1 = QLineEdit(self.tab_mission)
        self.lineEdit_vel_1.setObjectName(u"lineEdit_vel_1")
        self.lineEdit_vel_1.setGeometry(QRect(90, 280, 61, 26))
        self.lineEdit_vel_2 = QLineEdit(self.tab_mission)
        self.lineEdit_vel_2.setObjectName(u"lineEdit_vel_2")
        self.lineEdit_vel_2.setGeometry(QRect(160, 280, 61, 26))
        self.lineEdit_vel_3 = QLineEdit(self.tab_mission)
        self.lineEdit_vel_3.setObjectName(u"lineEdit_vel_3")
        self.lineEdit_vel_3.setGeometry(QRect(230, 280, 61, 26))
        self.lineEdit_vel_4 = QLineEdit(self.tab_mission)
        self.lineEdit_vel_4.setObjectName(u"lineEdit_vel_4")
        self.lineEdit_vel_4.setGeometry(QRect(300, 280, 61, 26))
        self.lineEdit_vel_5 = QLineEdit(self.tab_mission)
        self.lineEdit_vel_5.setObjectName(u"lineEdit_vel_5")
        self.lineEdit_vel_5.setGeometry(QRect(370, 280, 61, 26))
        self.lineEdit_acc_2 = QLineEdit(self.tab_mission)
        self.lineEdit_acc_2.setObjectName(u"lineEdit_acc_2")
        self.lineEdit_acc_2.setGeometry(QRect(160, 350, 61, 26))
        self.lineEdit_acc_5 = QLineEdit(self.tab_mission)
        self.lineEdit_acc_5.setObjectName(u"lineEdit_acc_5")
        self.lineEdit_acc_5.setGeometry(QRect(370, 350, 61, 26))
        self.lineEdit_acc_4 = QLineEdit(self.tab_mission)
        self.lineEdit_acc_4.setObjectName(u"lineEdit_acc_4")
        self.lineEdit_acc_4.setGeometry(QRect(300, 350, 61, 26))
        self.lineEdit_acc_1 = QLineEdit(self.tab_mission)
        self.lineEdit_acc_1.setObjectName(u"lineEdit_acc_1")
        self.lineEdit_acc_1.setGeometry(QRect(90, 350, 61, 26))
        self.lineEdit_acc_3 = QLineEdit(self.tab_mission)
        self.lineEdit_acc_3.setObjectName(u"lineEdit_acc_3")
        self.lineEdit_acc_3.setGeometry(QRect(230, 350, 61, 26))
        self.lineEdit_acc_0 = QLineEdit(self.tab_mission)
        self.lineEdit_acc_0.setObjectName(u"lineEdit_acc_0")
        self.lineEdit_acc_0.setGeometry(QRect(20, 350, 61, 26))
        self.label_4 = QLabel(self.tab_mission)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(100, 200, 31, 17))
        self.lineEdit_payload_x = QLineEdit(self.tab_mission)
        self.lineEdit_payload_x.setObjectName(u"lineEdit_payload_x")
        self.lineEdit_payload_x.setGeometry(QRect(160, 200, 71, 26))
        self.lineEdit_payload_y = QLineEdit(self.tab_mission)
        self.lineEdit_payload_y.setObjectName(u"lineEdit_payload_y")
        self.lineEdit_payload_y.setGeometry(QRect(240, 200, 71, 26))
        self.lineEdit_payload_z = QLineEdit(self.tab_mission)
        self.lineEdit_payload_z.setObjectName(u"lineEdit_payload_z")
        self.lineEdit_payload_z.setGeometry(QRect(320, 200, 71, 26))
        self.label_5 = QLabel(self.tab_mission)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(160, 170, 161, 17))
        self.btn_dyn_set = QPushButton(self.tab_mission)
        self.btn_dyn_set.setObjectName(u"btn_dyn_set")
        self.btn_dyn_set.setGeometry(QRect(340, 400, 91, 33))
        palette2 = QPalette()
        palette2.setBrush(QPalette.Active, QPalette.WindowText, brush)
        brush3 = QBrush(QColor(0, 210, 26, 255))
        brush3.setStyle(Qt.SolidPattern)
        palette2.setBrush(QPalette.Active, QPalette.Button, brush3)
        palette2.setBrush(QPalette.Active, QPalette.Text, brush)
        palette2.setBrush(QPalette.Active, QPalette.ButtonText, brush)
        palette2.setBrush(QPalette.Active, QPalette.Base, brush3)
        palette2.setBrush(QPalette.Active, QPalette.Window, brush3)
        palette2.setBrush(QPalette.Inactive, QPalette.WindowText, brush)
        palette2.setBrush(QPalette.Inactive, QPalette.Button, brush3)
        palette2.setBrush(QPalette.Inactive, QPalette.Text, brush)
        palette2.setBrush(QPalette.Inactive, QPalette.ButtonText, brush)
        palette2.setBrush(QPalette.Inactive, QPalette.Base, brush3)
        palette2.setBrush(QPalette.Inactive, QPalette.Window, brush3)
        palette2.setBrush(QPalette.Disabled, QPalette.WindowText, brush)
        palette2.setBrush(QPalette.Disabled, QPalette.Button, brush3)
        palette2.setBrush(QPalette.Disabled, QPalette.Text, brush)
        palette2.setBrush(QPalette.Disabled, QPalette.ButtonText, brush)
        palette2.setBrush(QPalette.Disabled, QPalette.Base, brush3)
        palette2.setBrush(QPalette.Disabled, QPalette.Window, brush3)
        self.btn_dyn_set.setPalette(palette2)
        self.btn_dyn_set.setFont(font2)
        self.btn_dyn_set.setStyleSheet(u"background-color:#00d21a;color:white;border-color: black;")
        self.lineEdit_jog_3 = QLineEdit(self.tab_mission)
        self.lineEdit_jog_3.setObjectName(u"lineEdit_jog_3")
        self.lineEdit_jog_3.setGeometry(QRect(230, 70, 61, 26))
        self.lineEdit_jog_1 = QLineEdit(self.tab_mission)
        self.lineEdit_jog_1.setObjectName(u"lineEdit_jog_1")
        self.lineEdit_jog_1.setGeometry(QRect(90, 70, 61, 26))
        self.lineEdit_jog_5 = QLineEdit(self.tab_mission)
        self.lineEdit_jog_5.setObjectName(u"lineEdit_jog_5")
        self.lineEdit_jog_5.setGeometry(QRect(370, 70, 61, 26))
        self.lineEdit_jog_4 = QLineEdit(self.tab_mission)
        self.lineEdit_jog_4.setObjectName(u"lineEdit_jog_4")
        self.lineEdit_jog_4.setGeometry(QRect(300, 70, 61, 26))
        self.lineEdit_jog_2 = QLineEdit(self.tab_mission)
        self.lineEdit_jog_2.setObjectName(u"lineEdit_jog_2")
        self.lineEdit_jog_2.setGeometry(QRect(160, 70, 61, 26))
        self.label_6 = QLabel(self.tab_mission)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(20, 30, 151, 17))
        self.lineEdit_jog_0 = QLineEdit(self.tab_mission)
        self.lineEdit_jog_0.setObjectName(u"lineEdit_jog_0")
        self.lineEdit_jog_0.setGeometry(QRect(20, 70, 61, 26))
        self.graphicsView = QGraphicsView(self.tab_mission)
        self.graphicsView.setObjectName(u"graphicsView")
        self.graphicsView.setGeometry(QRect(460, 20, 481, 381))
        self.btn_arm_plot = QPushButton(self.tab_mission)
        self.btn_arm_plot.setObjectName(u"btn_arm_plot")
        self.btn_arm_plot.setGeometry(QRect(340, 120, 91, 33))
        palette3 = QPalette()
        palette3.setBrush(QPalette.Active, QPalette.WindowText, brush)
        palette3.setBrush(QPalette.Active, QPalette.Button, brush3)
        palette3.setBrush(QPalette.Active, QPalette.Text, brush)
        palette3.setBrush(QPalette.Active, QPalette.ButtonText, brush)
        palette3.setBrush(QPalette.Active, QPalette.Base, brush3)
        palette3.setBrush(QPalette.Active, QPalette.Window, brush3)
        palette3.setBrush(QPalette.Inactive, QPalette.WindowText, brush)
        palette3.setBrush(QPalette.Inactive, QPalette.Button, brush3)
        palette3.setBrush(QPalette.Inactive, QPalette.Text, brush)
        palette3.setBrush(QPalette.Inactive, QPalette.ButtonText, brush)
        palette3.setBrush(QPalette.Inactive, QPalette.Base, brush3)
        palette3.setBrush(QPalette.Inactive, QPalette.Window, brush3)
        palette3.setBrush(QPalette.Disabled, QPalette.WindowText, brush)
        palette3.setBrush(QPalette.Disabled, QPalette.Button, brush3)
        palette3.setBrush(QPalette.Disabled, QPalette.Text, brush)
        palette3.setBrush(QPalette.Disabled, QPalette.ButtonText, brush)
        palette3.setBrush(QPalette.Disabled, QPalette.Base, brush3)
        palette3.setBrush(QPalette.Disabled, QPalette.Window, brush3)
        self.btn_arm_plot.setPalette(palette3)
        self.btn_arm_plot.setFont(font2)
        self.btn_arm_plot.setStyleSheet(u"background-color:#00d21a;color:white;border-color: black;")
        self.tabWidget.addTab(self.tab_mission, "")
        self.tab_state = QWidget()
        self.tab_state.setObjectName(u"tab_state")
        self.btn_dyn_space = QPushButton(self.tab_state)
        self.btn_dyn_space.setObjectName(u"btn_dyn_space")
        self.btn_dyn_space.setGeometry(QRect(710, 410, 201, 33))
        palette4 = QPalette()
        palette4.setBrush(QPalette.Active, QPalette.WindowText, brush)
        palette4.setBrush(QPalette.Active, QPalette.Button, brush3)
        palette4.setBrush(QPalette.Active, QPalette.Text, brush)
        palette4.setBrush(QPalette.Active, QPalette.ButtonText, brush)
        palette4.setBrush(QPalette.Active, QPalette.Base, brush3)
        palette4.setBrush(QPalette.Active, QPalette.Window, brush3)
        palette4.setBrush(QPalette.Inactive, QPalette.WindowText, brush)
        palette4.setBrush(QPalette.Inactive, QPalette.Button, brush3)
        palette4.setBrush(QPalette.Inactive, QPalette.Text, brush)
        palette4.setBrush(QPalette.Inactive, QPalette.ButtonText, brush)
        palette4.setBrush(QPalette.Inactive, QPalette.Base, brush3)
        palette4.setBrush(QPalette.Inactive, QPalette.Window, brush3)
        palette4.setBrush(QPalette.Disabled, QPalette.WindowText, brush)
        palette4.setBrush(QPalette.Disabled, QPalette.Button, brush3)
        palette4.setBrush(QPalette.Disabled, QPalette.Text, brush)
        palette4.setBrush(QPalette.Disabled, QPalette.ButtonText, brush)
        palette4.setBrush(QPalette.Disabled, QPalette.Base, brush3)
        palette4.setBrush(QPalette.Disabled, QPalette.Window, brush3)
        self.btn_dyn_space.setPalette(palette4)
        self.btn_dyn_space.setFont(font2)
        self.btn_dyn_space.setStyleSheet(u"background-color:#00d21a;color:white;border-color: black;")
        self.btn_dyn_axis_set = QPushButton(self.tab_state)
        self.btn_dyn_axis_set.setObjectName(u"btn_dyn_axis_set")
        self.btn_dyn_axis_set.setGeometry(QRect(140, 240, 91, 33))
        palette5 = QPalette()
        palette5.setBrush(QPalette.Active, QPalette.WindowText, brush)
        palette5.setBrush(QPalette.Active, QPalette.Button, brush3)
        palette5.setBrush(QPalette.Active, QPalette.Text, brush)
        palette5.setBrush(QPalette.Active, QPalette.ButtonText, brush)
        palette5.setBrush(QPalette.Active, QPalette.Base, brush3)
        palette5.setBrush(QPalette.Active, QPalette.Window, brush3)
        palette5.setBrush(QPalette.Inactive, QPalette.WindowText, brush)
        palette5.setBrush(QPalette.Inactive, QPalette.Button, brush3)
        palette5.setBrush(QPalette.Inactive, QPalette.Text, brush)
        palette5.setBrush(QPalette.Inactive, QPalette.ButtonText, brush)
        palette5.setBrush(QPalette.Inactive, QPalette.Base, brush3)
        palette5.setBrush(QPalette.Inactive, QPalette.Window, brush3)
        palette5.setBrush(QPalette.Disabled, QPalette.WindowText, brush)
        palette5.setBrush(QPalette.Disabled, QPalette.Button, brush3)
        palette5.setBrush(QPalette.Disabled, QPalette.Text, brush)
        palette5.setBrush(QPalette.Disabled, QPalette.ButtonText, brush)
        palette5.setBrush(QPalette.Disabled, QPalette.Base, brush3)
        palette5.setBrush(QPalette.Disabled, QPalette.Window, brush3)
        self.btn_dyn_axis_set.setPalette(palette5)
        self.btn_dyn_axis_set.setFont(font2)
        self.btn_dyn_axis_set.setStyleSheet(u"background-color:#00d21a;color:white;border-color: black;")
        self.lineEdit_axis_set = QLineEdit(self.tab_state)
        self.lineEdit_axis_set.setObjectName(u"lineEdit_axis_set")
        self.lineEdit_axis_set.setGeometry(QRect(40, 240, 61, 26))
        self.label_7 = QLabel(self.tab_state)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setGeometry(QRect(30, 200, 151, 17))
        self.lineEdit_payload_space = QLineEdit(self.tab_state)
        self.lineEdit_payload_space.setObjectName(u"lineEdit_payload_space")
        self.lineEdit_payload_space.setGeometry(QRect(30, 70, 71, 26))
        self.label_15 = QLabel(self.tab_state)
        self.label_15.setObjectName(u"label_15")
        self.label_15.setGeometry(QRect(30, 40, 67, 17))
        self.label_16 = QLabel(self.tab_state)
        self.label_16.setObjectName(u"label_16")
        self.label_16.setGeometry(QRect(110, 70, 31, 17))
        self.label_17 = QLabel(self.tab_state)
        self.label_17.setObjectName(u"label_17")
        self.label_17.setGeometry(QRect(170, 40, 161, 17))
        self.lineEdit_payload_y_space = QLineEdit(self.tab_state)
        self.lineEdit_payload_y_space.setObjectName(u"lineEdit_payload_y_space")
        self.lineEdit_payload_y_space.setGeometry(QRect(250, 70, 71, 26))
        self.lineEdit_payload_x_space = QLineEdit(self.tab_state)
        self.lineEdit_payload_x_space.setObjectName(u"lineEdit_payload_x_space")
        self.lineEdit_payload_x_space.setGeometry(QRect(170, 70, 71, 26))
        self.lineEdit_payload_z_space = QLineEdit(self.tab_state)
        self.lineEdit_payload_z_space.setObjectName(u"lineEdit_payload_z_space")
        self.lineEdit_payload_z_space.setGeometry(QRect(330, 70, 71, 26))
        self.btn_dyn_space_set = QPushButton(self.tab_state)
        self.btn_dyn_space_set.setObjectName(u"btn_dyn_space_set")
        self.btn_dyn_space_set.setGeometry(QRect(300, 120, 91, 33))
        palette6 = QPalette()
        palette6.setBrush(QPalette.Active, QPalette.WindowText, brush)
        palette6.setBrush(QPalette.Active, QPalette.Button, brush3)
        palette6.setBrush(QPalette.Active, QPalette.Text, brush)
        palette6.setBrush(QPalette.Active, QPalette.ButtonText, brush)
        palette6.setBrush(QPalette.Active, QPalette.Base, brush3)
        palette6.setBrush(QPalette.Active, QPalette.Window, brush3)
        palette6.setBrush(QPalette.Inactive, QPalette.WindowText, brush)
        palette6.setBrush(QPalette.Inactive, QPalette.Button, brush3)
        palette6.setBrush(QPalette.Inactive, QPalette.Text, brush)
        palette6.setBrush(QPalette.Inactive, QPalette.ButtonText, brush)
        palette6.setBrush(QPalette.Inactive, QPalette.Base, brush3)
        palette6.setBrush(QPalette.Inactive, QPalette.Window, brush3)
        palette6.setBrush(QPalette.Disabled, QPalette.WindowText, brush)
        palette6.setBrush(QPalette.Disabled, QPalette.Button, brush3)
        palette6.setBrush(QPalette.Disabled, QPalette.Text, brush)
        palette6.setBrush(QPalette.Disabled, QPalette.ButtonText, brush)
        palette6.setBrush(QPalette.Disabled, QPalette.Base, brush3)
        palette6.setBrush(QPalette.Disabled, QPalette.Window, brush3)
        self.btn_dyn_space_set.setPalette(palette6)
        self.btn_dyn_space_set.setFont(font2)
        self.btn_dyn_space_set.setStyleSheet(u"background-color:#00d21a;color:white;border-color: black;")
        self.graphicsView_3 = QGraphicsView(self.tab_state)
        self.graphicsView_3.setObjectName(u"graphicsView_3")
        self.graphicsView_3.setGeometry(QRect(460, 10, 481, 381))
        self.tabWidget.addTab(self.tab_state, "")
        self.tab = QWidget()
        self.tab.setObjectName(u"tab")
        self.tabWidget.addTab(self.tab, "")
        self.tab_other = QWidget()
        self.tab_other.setObjectName(u"tab_other")
        self.label_arm_picture = QLabel(self.tab_other)
        self.label_arm_picture.setObjectName(u"label_arm_picture")
        self.label_arm_picture.setGeometry(QRect(20, 80, 151, 141))
        self.label_arm_picture.setPixmap(QPixmap(u"../picture/teco_arm.png"))
        self.label_arm_picture.setScaledContents(True)
        self.label_arm_gif = QLabel(self.tab_other)
        self.label_arm_gif.setObjectName(u"label_arm_gif")
        self.label_arm_gif.setGeometry(QRect(30, 280, 151, 141))
        self.label_arm_gif.setPixmap(QPixmap(u"src/modbus/modbus/picture/teco_arm.png"))
        self.label_arm_gif.setScaledContents(True)
        self.tabWidget.addTab(self.tab_other, "")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 966, 23))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        self.tabWidget.setCurrentIndex(2)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Control Window", None))
#if QT_CONFIG(whatsthis)
        self.tabWidget.setWhatsThis(QCoreApplication.translate("MainWindow", u"<html><head/><body><p>dd</p></body></html>", None))
#endif // QT_CONFIG(whatsthis)
        self.lineEdit_ip_2.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.lineEdit_ip_3.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_arm), QCoreApplication.translate("MainWindow", u"Kinematics", None))
        self.btn_dynamics.setText(QCoreApplication.translate("MainWindow", u"Dynamics  Calculation", None))
        self.lineEdit_payload.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_vel_0.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Payload", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Angular velocity", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Angular acceleration", None))
        self.lineEdit_vel_1.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_vel_2.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_vel_3.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_vel_4.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_vel_5.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_acc_2.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_acc_5.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_acc_4.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_acc_1.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_acc_3.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_acc_0.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Kg", None))
        self.lineEdit_payload_x.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_payload_y.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_payload_z.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Payload Position", None))
        self.btn_dyn_set.setText(QCoreApplication.translate("MainWindow", u"Set", None))
        self.lineEdit_jog_3.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_jog_1.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_jog_5.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_jog_4.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_jog_2.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"Joint Angle", None))
        self.lineEdit_jog_0.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.btn_arm_plot.setText(QCoreApplication.translate("MainWindow", u"Plot", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_mission), QCoreApplication.translate("MainWindow", u"Dynamics", None))
        self.btn_dyn_space.setText(QCoreApplication.translate("MainWindow", u"Dynamics Space", None))
        self.btn_dyn_axis_set.setText(QCoreApplication.translate("MainWindow", u"Set", None))
        self.lineEdit_axis_set.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"select axis", None))
        self.lineEdit_payload_space.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.label_15.setText(QCoreApplication.translate("MainWindow", u"Payload", None))
        self.label_16.setText(QCoreApplication.translate("MainWindow", u"Kg", None))
        self.label_17.setText(QCoreApplication.translate("MainWindow", u"Payload Position", None))
        self.lineEdit_payload_y_space.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_payload_x_space.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.lineEdit_payload_z_space.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.btn_dyn_space_set.setText(QCoreApplication.translate("MainWindow", u"Set", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_state), QCoreApplication.translate("MainWindow", u"Dynamics Space", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), QCoreApplication.translate("MainWindow", u"Arm Data", None))
        self.label_arm_picture.setText("")
        self.label_arm_gif.setText("")
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_other), QCoreApplication.translate("MainWindow", u"Other", None))
    # retranslateUi

