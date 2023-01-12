#!/usr/bin/env python3
# coding: utf-8
import numpy
import rospy
from stl import mesh
from os import path
from urdf_parser_py.urdf import URDF, Robot
from interface_control.msg import specified_parameter_design, cal_cmd, optimal_random
import random
# TODO: fix stl conv 6dof urdf & 修改為可調整dof的方式
class stl_conv_urdf():
    def __init__(self, robot_name, robot_parameter):
        self.axis_2_length = 0.0
        self.axis_3_length = 0.0
        self.arm_weight = 0.0
        self.payload = 0.0
        self.DoF = 6
        self.pub_cmd = rospy.Publisher("/cal_command",cal_cmd, queue_size=10)
        self.pub_optimal_random = rospy.Publisher("/optimal_random",optimal_random, queue_size=10)
        self.sub_specified_parameter_design = rospy.Subscriber("/specified_parameter_design",specified_parameter_design, self.specified_parameter_design_callback)
        self.sub_interface_control = rospy.Subscriber("/cal_command",cal_cmd, self.interface_control_callback)
        self.robot_name = robot_name
        self.robot_parameter = robot_parameter
        self.robot_stl_axis_2 = None
        self.robot_stl_axis_3 = None
        self.robot_urdf = URDF.from_xml_file(path.dirname(path.realpath(__file__)) + "/urdf/" + robot_name + ".urdf")
        self.lines = []
        self.J3_line = 0
        self.J4_line = 0
        self.L2_line = 0
        self.L3_line = 0
        self.L2_volume = 0.0
        self.L3_volume = 0.0
        self.L2_cog = []
        self.L3_cog = []
        self.L2_inertia = []
        self.L3_inertia = []
        self.mesh_2_name = ""
        self.mesh_3_name = ""

        self.op_axis_2_length = 0.0
        self.op_axis_3_length = 0.0

        self.cal_cmd = cal_cmd()
        self.optimal_random = optimal_random()

    def stl_read_file(self):
        self.robot_stl_axis_2 = mesh.Mesh.from_file(path.dirname(path.realpath(__file__))+"/meshes/" + self.robot_name + "_2_"+ str(self.axis_2_length) + ".STL")
        self.robot_stl_axis_3 = mesh.Mesh.from_file(path.dirname(path.realpath(__file__))+"/meshes/" + self.robot_name + "_3_"+ str(self.axis_3_length) + ".STL")
        self.mesh_2_name = self.robot_name + "_2_"+ str(self.axis_2_length) + ".STL"
        self.mesh_3_name = self.robot_name + "_3_"+ str(self.axis_3_length) + ".STL"

        self.L2_volume, self.L2_cog, self.L2_inertia = self.robot_stl_axis_2.get_mass_properties()
        self.L2_volume = self.L2_volume*1000
        self.L2_inertia = self.L2_inertia*1000
        print("============================================================")
        print("Volume                                  = {0}".format(self.L2_volume))
        print("Position of the center of gravity (COG) = {0}".format(self.L2_cog))
        print("Inertia matrix at expressed at the COG  = {0}".format(self.L2_inertia[0,:]))
        print("                                          {0}".format(self.L2_inertia[1,:]))
        print("                                          {0}".format(self.L2_inertia[2,:]))

        self.L3_volume, self.L3_cog, self.L3_inertia = self.robot_stl_axis_3.get_mass_properties()
        self.L3_volume = self.L3_volume*1000
        self.L3_inertia = self.L3_inertia*1000
        print("============================================================")
        print("Volume                                  = {0}".format(self.L3_volume))
        print("Position of the center of gravity (COG) = {0}".format(self.L3_cog))
        print("Inertia matrix at expressed at the COG  = {0}".format(self.L3_inertia[0,:]))
        print("                                          {0}".format(self.L3_inertia[1,:]))
        print("                                          {0}".format(self.L3_inertia[2,:]))
        print("============================================================")
    def read_check_urdf(self):
        
        with open(path.dirname(path.realpath(__file__)) + "/urdf/" + self.robot_name + ".urdf",'r',encoding='utf-8') as urdf_config:
            self.lines = urdf_config.readlines()
            size = urdf_config.read()
            # print(urdf_config.read())
            flen=len(self.lines)
            print("line count:", flen)

            for i in range(flen):
                if self.lines[i].startswith("    name=\"2\"") and self.lines[i+2].startswith("      <origin"):
                    self.L2_line = i+1
                    print("link 2 line number:", self.L2_line)

                elif self.lines[i].startswith("    name=\"3\"") and self.lines[i+2].startswith("      <origin"):
                    self.L3_line = i+1
                    print("link 3 line number:", self.L3_line)

                elif self.lines[i].startswith("    name=\"j3\"") and self.lines[i+2].startswith("    <origin"):
                    self.J3_line = i+1
                    print("joint 3 line number:", self.J3_line)

                elif self.lines[i].startswith("    name=\"j4\"") and self.lines[i+2].startswith("    <origin"):
                    self.J4_line = i+1
                    print("joint 4 line number:", self.J4_line)
                    print("read urdf data no problem")
                    print("============================================================")
                    break
                else:
                    continue
    

    def data_write_urdf(self):
        with open(path.dirname(path.realpath(__file__)) + "/urdf/" + self.robot_name + ".urdf",'w',encoding='utf-8') as urdf_config:
            cog_lines = int(self.L2_line + 2)
            mass_lines = int(self.L2_line + 5)
            inertia_lines = int(self.L2_line + 7)
            mesh_lines = int(self.L2_line + 20)
            joint_pos = int(self.J3_line + 2)
            
            # self.axis_2_length = 281.5 - self.axis_2_length*10
            # TODO: fixed -------
            self.axis_2_length = 408.000000000056 - self.axis_2_length*10
            joint3_pos_y = 0.408000000000056 - self.axis_2_length*0.001
            self.lines[cog_lines] = ("        xyz=\"{0[0]} {0[1]} {0[2]}\"\n".format(self.L2_cog))
            self.lines[mass_lines] = ("        value=\"{0}\"  />\n".format(self.L2_volume))
            self.lines[inertia_lines] = ("        ixx=\"{0[0][0]}\"\n".format(self.L2_inertia))
            self.lines[inertia_lines+1] = ("        ixy=\"{0[0][1]}\"\n".format(self.L2_inertia))
            self.lines[inertia_lines+2] = ("        ixz=\"{0[0][2]}\"\n".format(self.L2_inertia))
            self.lines[inertia_lines+3] = ("        iyy=\"{0[1][1]}\"\n".format(self.L2_inertia))
            self.lines[inertia_lines+4] = ("        iyz=\"{0[1][2]}\"\n".format(self.L2_inertia))
            self.lines[inertia_lines+5] = ("        izz=\"{0[2][2]}\" />\n".format(self.L2_inertia))
            self.lines[mesh_lines] = ("          filename=\"package://dynamics/src/dynamics/meshes/{0}\" />\n".format(self.mesh_2_name))
            self.lines[joint_pos] = ("      xyz=\"0 {0} 0\"\n".format(joint3_pos_y))
            for data in self.lines:
                urdf_config.write(data)
            urdf_config.flush()

        with open(path.dirname(path.realpath(__file__)) + "/urdf/" + self.robot_name + ".urdf",'w',encoding='utf-8') as urdf_config:
            cog_lines = int(self.L3_line + 2)
            mass_lines = int(self.L3_line + 5)
            inertia_lines = int(self.L3_line + 7)
            mesh_lines = int(self.L3_line + 20)
            joint_pos = int(self.J4_line + 2)
            # TODO: fixed -------
            self.axis_3_length = 372.50368954 - self.axis_3_length*10
            joint4_pos_y = 0.37250368954 - self.axis_3_length*0.001
            self.lines[cog_lines] = ("        xyz=\"{0[0]} {0[1]} {0[2]}\"\n".format(self.L3_cog))
            self.lines[mass_lines] = ("        value=\"{0}\"  />\n".format(self.L3_volume))
            self.lines[inertia_lines] = ("        ixx=\"{0[0][0]}\"\n".format(self.L3_inertia))
            self.lines[inertia_lines+1] = ("        ixy=\"{0[0][1]}\"\n".format(self.L3_inertia))
            self.lines[inertia_lines+2] = ("        ixz=\"{0[0][2]}\"\n".format(self.L3_inertia))
            self.lines[inertia_lines+3] = ("        iyy=\"{0[1][1]}\"\n".format(self.L3_inertia))
            self.lines[inertia_lines+4] = ("        iyz=\"{0[1][2]}\"\n".format(self.L3_inertia))
            self.lines[inertia_lines+5] = ("        izz=\"{0[2][2]}\" />\n".format(self.L3_inertia))
            self.lines[mesh_lines] = ("          filename=\"package://dynamics/src/dynamics/meshes/{0}\" />\n".format(self.mesh_3_name))
            self.lines[joint_pos] = ("      xyz=\"0 {0} 0.0203999999999699\"\n".format(joint4_pos_y))
            for data in self.lines:
                urdf_config.write(data)
            urdf_config.flush()
    def random_generate_write_urdf(self):
        # TODO: fixed -------
        your_mesh = mesh.Mesh.from_file(path.dirname(path.realpath(__file__)) + "/meshes/" +'random_3_25.0.STL')
        volume_1, cog_1, inertia_1 = your_mesh.get_mass_properties()
        your_mesh = mesh.Mesh.from_file(path.dirname(path.realpath(__file__)) + "/meshes/" +'random_3_30.0.STL')
        volume_2, cog_2, inertia_2 = your_mesh.get_mass_properties()
        vol = volume_2 - volume_1
        cog = cog_2 - cog_1
        inertia = inertia_2 - inertia_1

        # ===================================
        self.axis_2_length = 35.0
        self.axis_3_length = 35.0
        self.robot_stl_axis_2 = mesh.Mesh.from_file(path.dirname(path.realpath(__file__))+"/meshes/" + self.robot_name + "_2_"+ str(self.axis_2_length) + ".STL")
        self.robot_stl_axis_3 = mesh.Mesh.from_file(path.dirname(path.realpath(__file__))+"/meshes/" + self.robot_name + "_3_"+ str(self.axis_3_length) + ".STL")
        self.mesh_2_name = self.robot_name + "_2_"+ str(self.axis_2_length) + ".STL"
        self.mesh_3_name = self.robot_name + "_3_"+ str(self.axis_3_length) + ".STL"

        self.L2_volume, self.L2_cog, self.L2_inertia = self.robot_stl_axis_2.get_mass_properties()
        # random axis 2 length
        # TODO: fixed -------
        random_axis_2_length = random.uniform(self.axis_2_length - 15.0, self.axis_2_length + 15.0)
        print("random_axis_2_length                    = {0}".format(random_axis_2_length))
        diff_axis_2_length = random_axis_2_length - self.axis_2_length
        self.axis_2_length = random_axis_2_length
        self.op_axis_2_length = random_axis_2_length
        self.L2_volume = self.L2_volume + vol*(diff_axis_2_length/5)
        self.L2_cog = self.L2_cog + cog*(diff_axis_2_length/5)
        self.L2_inertia = self.L2_inertia + inertia*(diff_axis_2_length/5)

        self.L2_volume = self.L2_volume*1000
        self.L2_inertia = self.L2_inertia*1000
        print("============================================================")
        print("Volume                                  = {0}".format(self.L2_volume))
        print("Position of the center of gravity (COG) = {0}".format(self.L2_cog))
        print("Inertia matrix at expressed at the COG  = {0}".format(self.L2_inertia[0,:]))
        print("                                          {0}".format(self.L2_inertia[1,:]))
        print("                                          {0}".format(self.L2_inertia[2,:]))

        self.L3_volume, self.L3_cog, self.L3_inertia = self.robot_stl_axis_3.get_mass_properties()
        # random axis 3 length
        # TODO: fixed -------
        random_axis_3_length = random.uniform(self.axis_3_length - 15.0, self.axis_3_length + 15.0)
        print("random_axis_3_length                    = {0}".format(random_axis_3_length))
        diff_axis_3_length = random_axis_3_length - self.axis_3_length
        self.axis_3_length = random_axis_3_length
        self.op_axis_3_length = random_axis_3_length
        self.L3_volume = self.L3_volume + vol*(diff_axis_3_length/5)
        self.L3_cog = self.L3_cog + cog*(diff_axis_3_length/5)
        self.L3_inertia = self.L3_inertia + inertia*(diff_axis_3_length/5)

        self.L3_volume = self.L3_volume*1000
        self.L3_inertia = self.L3_inertia*1000
        print("============================================================")
        print("Volume                                  = {0}".format(self.L3_volume))
        print("Position of the center of gravity (COG) = {0}".format(self.L3_cog))
        print("Inertia matrix at expressed at the COG  = {0}".format(self.L3_inertia[0,:]))
        print("                                          {0}".format(self.L3_inertia[1,:]))
        print("                                          {0}".format(self.L3_inertia[2,:]))
        print("============================================================")
        # ===================================
        self.read_check_urdf()
        self.data_write_urdf()

    def task_set(self):
        pass
    # Generate information specified on the interface
    def specified_parameter_design_callback(self, data):
        self.axis_2_length = data.axis_2_length
        self.axis_3_length = data.axis_3_length
        self.arm_weight = data.arm_weight
        self.payload = data.payload
        self.radius = data.radius  # 半径
        self.DoF = data.DoF
        rospy.loginfo("I heard axis_2_length is %s", self.axis_2_length)
        rospy.loginfo("I heard axis_3_length is %s", self.axis_3_length)
        rospy.loginfo("I heard arm_weight is %s", self.arm_weight)
        rospy.loginfo("I heard payload is %s", self.payload)
        rospy.loginfo("I heard radius is %s", self.radius)
        rospy.loginfo("I heard DoF is %s", self.DoF)

        # self.stl_read_file()
        # self.read_check_urdf()
        # self.data_write_urdf()
        self.generate_write_urdf()
        self.pub_cmd.publish(7) # rebuild robot
    # Randomly generated information on the interface
    def interface_control_callback(self,data):
        cmd = data.cmd
        # rebuild robot modules
        if cmd == 20:
            self.random_generate_write_urdf()
            self.pub_cmd.publish(7) # rebuild robot
        # rebuild robot & motor match
        elif cmd == 21:
            self.random_generate_write_urdf()
            # self.pub_cmd.publish(8)
            # TODO: return axis 2, 3 random length
            self.optimal_random.axis_2_length = self.op_axis_2_length
            self.optimal_random.axis_3_length = self.op_axis_3_length
            self.pub_optimal_random.publish(self.optimal_random)

    # generated information on the interface
    def generate_write_urdf(self):
        # 比較求取等差間距
        # TODO: fixed -------
        your_mesh = mesh.Mesh.from_file(path.dirname(path.realpath(__file__)) + "/meshes/" +'random_3_25.0.STL')
        volume_1, cog_1, inertia_1 = your_mesh.get_mass_properties()
        your_mesh = mesh.Mesh.from_file(path.dirname(path.realpath(__file__)) + "/meshes/" +'random_3_30.0.STL')
        volume_2, cog_2, inertia_2 = your_mesh.get_mass_properties()
        vol = volume_2 - volume_1
        cog = cog_2 - cog_1
        inertia = inertia_2 - inertia_1

        # =================================== 設定標準長度
        # TODO: fixed -------
        axis_2_length = 35.0
        axis_3_length = 35.0
        self.robot_stl_axis_2 = mesh.Mesh.from_file(path.dirname(path.realpath(__file__))+"/meshes/" + self.robot_name + "_2_"+ str(axis_2_length) + ".STL")
        self.robot_stl_axis_3 = mesh.Mesh.from_file(path.dirname(path.realpath(__file__))+"/meshes/" + self.robot_name + "_3_"+ str(axis_3_length) + ".STL")
        self.mesh_2_name = self.robot_name + "_2_"+ str(self.axis_2_length) + ".STL"
        self.mesh_3_name = self.robot_name + "_3_"+ str(self.axis_3_length) + ".STL"

        self.L2_volume, self.L2_cog, self.L2_inertia = self.robot_stl_axis_2.get_mass_properties()
        # random axis 2 length
        # random_axis_2_length = random.uniform(self.axis_2_length - 15.0, self.axis_2_length + 15.0)
        # print("random_axis_2_length                    = {0}".format(random_axis_2_length))
        diff_axis_2_length = self.axis_2_length - axis_2_length
        # self.axis_2_length = self.axis_2_length
        self.op_axis_2_length = self.axis_3_length
        # TODO: fixed -------
        self.L2_volume = self.L2_volume + vol*(diff_axis_2_length/5)
        self.L2_cog = self.L2_cog + cog*(diff_axis_2_length/5)
        self.L2_inertia = self.L2_inertia + inertia*(diff_axis_2_length/5)

        self.L2_volume = self.L2_volume*1000
        self.L2_inertia = self.L2_inertia*1000
        print("============================================================")
        print("Volume                                  = {0}".format(self.L2_volume))
        print("Position of the center of gravity (COG) = {0}".format(self.L2_cog))
        print("Inertia matrix at expressed at the COG  = {0}".format(self.L2_inertia[0,:]))
        print("                                          {0}".format(self.L2_inertia[1,:]))
        print("                                          {0}".format(self.L2_inertia[2,:]))

        self.L3_volume, self.L3_cog, self.L3_inertia = self.robot_stl_axis_3.get_mass_properties()
        # random axis 3 length
        # random_axis_3_length = random.uniform(self.axis_3_length - 15.0, self.axis_3_length + 15.0)
        # print("random_axis_3_length                    = {0}".format(random_axis_3_length))
        diff_axis_3_length = self.axis_3_length - axis_3_length
        # self.axis_3_length = self.axis_3_length
        self.op_axis_3_length = self.axis_3_length
        # TODO: fixed -------
        self.L3_volume = self.L3_volume + vol*(diff_axis_3_length/5)
        self.L3_cog = self.L3_cog + cog*(diff_axis_3_length/5)
        self.L3_inertia = self.L3_inertia + inertia*(diff_axis_3_length/5)

        self.L3_volume = self.L3_volume*1000
        self.L3_inertia = self.L3_inertia*1000
        print("============================================================")
        print("Volume                                  = {0}".format(self.L3_volume))
        print("Position of the center of gravity (COG) = {0}".format(self.L3_cog))
        print("Inertia matrix at expressed at the COG  = {0}".format(self.L3_inertia[0,:]))
        print("                                          {0}".format(self.L3_inertia[1,:]))
        print("                                          {0}".format(self.L3_inertia[2,:]))
        print("============================================================")
        # ===================================
        self.read_check_urdf()
        self.data_write_urdf()

    # for opt generated information on the interface 標準軸長
    def opt_generate_write_urdf(self):
        # 比較求取等差間距
        # TODO: fixed -------
        your_mesh = mesh.Mesh.from_file(path.dirname(path.realpath(__file__)) + "/meshes/" +'random_3_25.0.STL')
        volume_1, cog_1, inertia_1 = your_mesh.get_mass_properties()
        your_mesh = mesh.Mesh.from_file(path.dirname(path.realpath(__file__)) + "/meshes/" +'random_3_30.0.STL')
        volume_2, cog_2, inertia_2 = your_mesh.get_mass_properties()
        vol = volume_2 - volume_1
        cog = cog_2 - cog_1
        inertia = inertia_2 - inertia_1
        # =================================== 設定標準長度
        # TODO: fixed -------
        axis_2_length = 35.0
        axis_3_length = 35.0
        self.robot_stl_axis_2 = mesh.Mesh.from_file(path.dirname(path.realpath(__file__))+"/meshes/" + self.robot_name + "_2_"+ str(axis_2_length) + ".STL")
        self.robot_stl_axis_3 = mesh.Mesh.from_file(path.dirname(path.realpath(__file__))+"/meshes/" + self.robot_name + "_3_"+ str(axis_3_length) + ".STL")
        self.mesh_2_name = self.robot_name + "_2_"+ str(axis_2_length) + ".STL"
        self.mesh_3_name = self.robot_name + "_3_"+ str(axis_3_length) + ".STL"
        self.L2_volume, self.L2_cog, self.L2_inertia = self.robot_stl_axis_2.get_mass_properties()
        # specified axis 2 length
        specified_axis_2_length = axis_2_length
        diff_axis_2_length = specified_axis_2_length - self.axis_2_length
        self.axis_2_length = specified_axis_2_length
        self.op_axis_2_length = specified_axis_2_length
        # TODO: fixed -------
        self.L2_volume = self.L2_volume + vol*(diff_axis_2_length/5)
        self.L2_cog = self.L2_cog + cog*(diff_axis_2_length/5)
        self.L2_inertia = self.L2_inertia + inertia*(diff_axis_2_length/5)
        self.L2_volume = self.L2_volume*1000
        self.L2_inertia = self.L2_inertia*1000
        self.L3_volume, self.L3_cog, self.L3_inertia = self.robot_stl_axis_3.get_mass_properties()
        # specified axis 3 length
        specified_axis_3_length = axis_3_length
        diff_axis_3_length = specified_axis_3_length - self.axis_3_length
        self.axis_3_length = specified_axis_3_length
        self.op_axis_3_length = specified_axis_3_length
        # TODO: fixed -------
        self.L3_volume = self.L3_volume + vol*(diff_axis_3_length/5)
        self.L3_cog = self.L3_cog + cog*(diff_axis_3_length/5)
        self.L3_inertia = self.L3_inertia + inertia*(diff_axis_3_length/5)

        self.L3_volume = self.L3_volume*1000
        self.L3_inertia = self.L3_inertia*1000
        self.opt_read_check_urdf()
        self.opt_data_write_urdf(axis_2_length,axis_3_length)
     
    def opt_random_generate_write_urdf(self):
        # TODO: fixed -------
        your_mesh = mesh.Mesh.from_file(path.dirname(path.realpath(__file__)) + "/meshes/" +'random_3_25.0.STL')
        volume_1, cog_1, inertia_1 = your_mesh.get_mass_properties()
        your_mesh = mesh.Mesh.from_file(path.dirname(path.realpath(__file__)) + "/meshes/" +'random_3_30.0.STL')
        volume_2, cog_2, inertia_2 = your_mesh.get_mass_properties()
        vol = volume_2 - volume_1
        cog = cog_2 - cog_1
        inertia = inertia_2 - inertia_1

        # ===================================
        self.axis_2_length = 35.0
        self.axis_3_length = 35.0
        self.robot_stl_axis_2 = mesh.Mesh.from_file(path.dirname(path.realpath(__file__))+"/meshes/" + self.robot_name + "_2_"+ str(self.axis_2_length) + ".STL")
        self.robot_stl_axis_3 = mesh.Mesh.from_file(path.dirname(path.realpath(__file__))+"/meshes/" + self.robot_name + "_3_"+ str(self.axis_3_length) + ".STL")
        self.mesh_2_name = self.robot_name + "_2_"+ str(self.axis_2_length) + ".STL"
        self.mesh_3_name = self.robot_name + "_3_"+ str(self.axis_3_length) + ".STL"

        self.L2_volume, self.L2_cog, self.L2_inertia = self.robot_stl_axis_2.get_mass_properties()
        # random axis 2 length
        # TODO: fixed -------
        random_axis_2_length = random.uniform(self.axis_2_length - 15.0, self.axis_2_length + 15.0)
        # print("random_axis_2_length                    = {0}".format(random_axis_2_length))
        diff_axis_2_length = random_axis_2_length - self.axis_2_length
        self.axis_2_length = random_axis_2_length
        self.op_axis_2_length = random_axis_2_length
        # TODO: fixed -------
        self.L2_volume = self.L2_volume + vol*(diff_axis_2_length/5)
        self.L2_cog = self.L2_cog + cog*(diff_axis_2_length/5)
        self.L2_inertia = self.L2_inertia + inertia*(diff_axis_2_length/5)

        self.L2_volume = self.L2_volume*1000
        self.L2_inertia = self.L2_inertia*1000
        
        self.L3_volume, self.L3_cog, self.L3_inertia = self.robot_stl_axis_3.get_mass_properties()
        # random axis 3 length
        # TODO: fixed -------
        random_axis_3_length = random.uniform(self.axis_3_length - 15.0, self.axis_3_length + 15.0)
        # print("random_axis_3_length                    = {0}".format(random_axis_3_length))
        diff_axis_3_length = random_axis_3_length - self.axis_3_length
        self.axis_3_length = random_axis_3_length
        self.op_axis_3_length = random_axis_3_length
        # TODO: fixed -------
        self.L3_volume = self.L3_volume + vol*(diff_axis_3_length/5)
        self.L3_cog = self.L3_cog + cog*(diff_axis_3_length/5)
        self.L3_inertia = self.L3_inertia + inertia*(diff_axis_3_length/5)

        self.L3_volume = self.L3_volume*1000
        self.L3_inertia = self.L3_inertia*1000
        # ===================================
        self.opt_read_check_urdf()
        self.opt_data_write_urdf(random_axis_2_length,random_axis_3_length)
        
        return random_axis_2_length, random_axis_3_length
    # for 6 dof generated information on the interface specified length 2 , 3 axis
    def opt_data_write_urdf(self,L2,L3):
        with open(path.dirname(path.realpath(__file__)) + "/urdf/" + self.robot_name + ".urdf",'w',encoding='utf-8') as urdf_config:
            cog_lines = int(self.L2_line + 2)
            mass_lines = int(self.L2_line + 5)
            inertia_lines = int(self.L2_line + 7)
            mesh_lines = int(self.L2_line + 20)
            joint_pos = int(self.J3_line + 2)
            
            # self.axis_2_length = 281.5 - self.axis_2_length*10
            # TODO: fixed -------
            L2 = 408.000000000056 - L2*10
            joint3_pos_y = 0.408000000000056 - L2*0.001
            self.lines[cog_lines] = ("        xyz=\"{0[0]} {0[1]} {0[2]}\"\n".format(self.L2_cog))
            self.lines[mass_lines] = ("        value=\"{0}\"  />\n".format(self.L2_volume))
            self.lines[inertia_lines] = ("        ixx=\"{0[0][0]}\"\n".format(self.L2_inertia))
            self.lines[inertia_lines+1] = ("        ixy=\"{0[0][1]}\"\n".format(self.L2_inertia))
            self.lines[inertia_lines+2] = ("        ixz=\"{0[0][2]}\"\n".format(self.L2_inertia))
            self.lines[inertia_lines+3] = ("        iyy=\"{0[1][1]}\"\n".format(self.L2_inertia))
            self.lines[inertia_lines+4] = ("        iyz=\"{0[1][2]}\"\n".format(self.L2_inertia))
            self.lines[inertia_lines+5] = ("        izz=\"{0[2][2]}\" />\n".format(self.L2_inertia))
            self.lines[mesh_lines] = ("          filename=\"package://dynamics/src/dynamics/meshes/{0}\" />\n".format(self.mesh_2_name))
            self.lines[joint_pos] = ("      xyz=\"0 {0} 0\"\n".format(joint3_pos_y))
            for data in self.lines:
                urdf_config.write(data)
            urdf_config.flush()

        with open(path.dirname(path.realpath(__file__)) + "/urdf/" + self.robot_name + ".urdf",'w',encoding='utf-8') as urdf_config:
            cog_lines = int(self.L3_line + 2)
            mass_lines = int(self.L3_line + 5)
            inertia_lines = int(self.L3_line + 7)
            mesh_lines = int(self.L3_line + 20)
            joint_pos = int(self.J4_line + 2)
            # TODO: fixed -------
            L3 = 372.50368954 - L3*10
            joint4_pos_y = 0.37250368954 - L3*0.001
            self.lines[cog_lines] = ("        xyz=\"{0[0]} {0[1]} {0[2]}\"\n".format(self.L3_cog))
            self.lines[mass_lines] = ("        value=\"{0}\"  />\n".format(self.L3_volume))
            self.lines[inertia_lines] = ("        ixx=\"{0[0][0]}\"\n".format(self.L3_inertia))
            self.lines[inertia_lines+1] = ("        ixy=\"{0[0][1]}\"\n".format(self.L3_inertia))
            self.lines[inertia_lines+2] = ("        ixz=\"{0[0][2]}\"\n".format(self.L3_inertia))
            self.lines[inertia_lines+3] = ("        iyy=\"{0[1][1]}\"\n".format(self.L3_inertia))
            self.lines[inertia_lines+4] = ("        iyz=\"{0[1][2]}\"\n".format(self.L3_inertia))
            self.lines[inertia_lines+5] = ("        izz=\"{0[2][2]}\" />\n".format(self.L3_inertia))
            self.lines[mesh_lines] = ("          filename=\"package://dynamics/src/dynamics/meshes/{0}\" />\n".format(self.mesh_3_name))
            self.lines[joint_pos] = ("      xyz=\"0 {0} 0.0203999999999699\"\n".format(joint4_pos_y))
            for data in self.lines:
                urdf_config.write(data)
            urdf_config.flush()


    def opt_read_check_urdf(self):
        
        with open(path.dirname(path.realpath(__file__)) + "/urdf/" + self.robot_name + ".urdf",'r',encoding='utf-8') as urdf_config:
            self.lines = urdf_config.readlines()
            size = urdf_config.read()
            # print(urdf_config.read())
            flen=len(self.lines)
            for i in range(flen):
                if self.lines[i].startswith("    name=\"2\"") and self.lines[i+2].startswith("      <origin"):
                    self.L2_line = i+1
                elif self.lines[i].startswith("    name=\"3\"") and self.lines[i+2].startswith("      <origin"):
                    self.L3_line = i+1
                elif self.lines[i].startswith("    name=\"j3\"") and self.lines[i+2].startswith("    <origin"):
                    self.J3_line = i+1
                elif self.lines[i].startswith("    name=\"j4\"") and self.lines[i+2].startswith("    <origin"):
                    self.J4_line = i+1
                    break
                else:
                    continue
    def specified_generate_write_urdf(self,L2, L3):
        # 比較求取等差間距
        # TODO: fixed -------
        your_mesh = mesh.Mesh.from_file(path.dirname(path.realpath(__file__)) + "/meshes/" +'random_3_25.0.STL')
        volume_1, cog_1, inertia_1 = your_mesh.get_mass_properties()
        your_mesh = mesh.Mesh.from_file(path.dirname(path.realpath(__file__)) + "/meshes/" +'random_3_30.0.STL')
        volume_2, cog_2, inertia_2 = your_mesh.get_mass_properties()
        vol = volume_2 - volume_1
        cog = cog_2 - cog_1
        inertia = inertia_2 - inertia_1
        # ===================================
        # TODO: fixed -------
        self.axis_2_length = 35.0
        self.axis_3_length = 35.0
        self.robot_stl_axis_2 = mesh.Mesh.from_file(path.dirname(path.realpath(__file__))+"/meshes/" + self.robot_name + "_2_"+ str(self.axis_2_length) + ".STL")
        self.robot_stl_axis_3 = mesh.Mesh.from_file(path.dirname(path.realpath(__file__))+"/meshes/" + self.robot_name + "_3_"+ str(self.axis_3_length) + ".STL")
        self.mesh_2_name = self.robot_name + "_2_"+ str(self.axis_2_length) + ".STL"
        self.mesh_3_name = self.robot_name + "_3_"+ str(self.axis_3_length) + ".STL"
        self.L2_volume, self.L2_cog, self.L2_inertia = self.robot_stl_axis_2.get_mass_properties()
        # specified axis 2 length
        specified_axis_2_length = L2
        diff_axis_2_length = specified_axis_2_length - self.axis_2_length
        self.axis_2_length = specified_axis_2_length
        self.op_axis_2_length = specified_axis_2_length
        # TODO: fixed -------
        self.L2_volume = self.L2_volume + vol*(diff_axis_2_length/5)
        self.L2_cog = self.L2_cog + cog*(diff_axis_2_length/5)
        self.L2_inertia = self.L2_inertia + inertia*(diff_axis_2_length/5)
        self.L2_volume = self.L2_volume*1000
        self.L2_inertia = self.L2_inertia*1000
        self.L3_volume, self.L3_cog, self.L3_inertia = self.robot_stl_axis_3.get_mass_properties()
        # specified axis 3 length
        specified_axis_3_length = L3
        diff_axis_3_length = specified_axis_3_length - self.axis_3_length
        self.axis_3_length = specified_axis_3_length
        self.op_axis_3_length = specified_axis_3_length
        # TODO: fixed -------
        self.L3_volume = self.L3_volume + vol*(diff_axis_3_length/5)
        self.L3_cog = self.L3_cog + cog*(diff_axis_3_length/5)
        self.L3_inertia = self.L3_inertia + inertia*(diff_axis_3_length/5)

        self.L3_volume = self.L3_volume*1000
        self.L3_inertia = self.L3_inertia*1000
        self.opt_read_check_urdf()
        self.opt_data_write_urdf(L2,L3)

    # Randomly generated information on the interface
    
if __name__ == '__main__':

    rospy.init_node('stl_cal')
    # TODO: fixed -------
    stl_cal = stl_conv_urdf("random","test")

    while not rospy.is_shutdown():
        stl_cal.task_set()


    