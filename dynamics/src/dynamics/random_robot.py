#!/usr/bin/env python3
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import spatialmath as sm
from urdf_parser_py.urdf import URDF
import os
class RandomRobot(DHRobot):
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
        self.length_1 = 0.1
        self.length_2 = 0.1
        self.length_3 = 0.1

        robot = URDF.from_xml_file(os.path.dirname(os.path.realpath(__file__))+"/urdf"+"/random.urdf")
        if symbolic:
            import spatialmath.base.symbolic as sym
            zero = sym.zero()
            pi = sym.pi()
        else:
            from math import pi
            zero = 0.0

        deg = pi / 180
        inch = 0.0254

        self.length_1 = robot.joints[3].origin.xyz[1]
        self.length_2 = robot.joints[4].origin.xyz[1]
        self.length_3 = robot.joints[5].origin.xyz[2]
        # robot length values (metres)
        # a = [0, -0.314, -0.284, 0, 0, 0] #m # teco
        a = [0, -robot.joints[3].origin.xyz[1], -robot.joints[4].origin.xyz[1], 0, 0, 0]
        # a = [0, -j3.y, -j4.y, 0, 0, 0]
        # d = [0.1301, 0, 0, 0.1145, 0.090, 0.048] #m # teco 
        d = [robot.joints[1].origin.xyz[2],
            0,
            0,
            robot.joints[2].origin.xyz[1]-robot.joints[4].origin.xyz[2], 
            robot.joints[5].origin.xyz[2],
            robot.joints[6].origin.xyz[2]
        ]
        # d = [j1.z, 0, 0, j2.y-j4.z , j5.z, j6.z] #m
        alpha = [pi/2, zero, zero, pi/2, -pi/2, zero]

        
        # mass data, no inertia available
        # mass = [3.7000, 8.3930, 2.33, 1.2190, 1.2190, 0.1897]
        mass = [robot.links[2].inertial.mass, 
                robot.links[3].inertial.mass,
                robot.links[4].inertial.mass,
                robot.links[5].inertial.mass,
                robot.links[6].inertial.mass,
                robot.links[7].inertial.mass
            ]
        
        G= [-80,-80,-80,-50,-50,-50]   # gear ratio
        G= [-1,-1,-1,-1,-1,-1]   # gear ratio
        # B = [[10,10], [10,10], [10,10], [10,10], [10,10], [10,10]]
        B = 10.0
        # center_of_mass = [
        #         [-1.55579201081481E-05, 0.00265005484815443, -0.00640979059142413],
        #         [4.90637956589368E-11, 0.205571973027702, -0.003359898058563426],
        #         [-9.75479428030767E-05, 0.271025707847572, 0.111573843205116],
        #         [-0.000181761828397664, 0.00219045749084071, -0.000800397394362884],
        #         [-0.000192919655058627, -0.00232492307126431, 0.00352418959262345],
        #         [-4.4856E-13 ,0 ,0.025]
        #     ]

        center_of_mass = [
                robot.links[2].inertial.origin.xyz,
                robot.links[3].inertial.origin.xyz,
                robot.links[4].inertial.origin.xyz,
                robot.links[5].inertial.origin.xyz,
                robot.links[6].inertial.origin.xyz,
                robot.links[7].inertial.origin.xyz
            ]
        # signed from a 3 3 matrix or a 6-element vector interpretted as Ixx Iyy Izz Ixy Iyz Ixz
        inertia = [
                [robot.links[2].inertial.inertia.ixx, robot.links[2].inertial.inertia.iyy, robot.links[2].inertial.inertia.izz, robot.links[2].inertial.inertia.ixy,robot.links[2].inertial.inertia.iyz, robot.links[2].inertial.inertia.ixz],
                [robot.links[3].inertial.inertia.ixx, robot.links[3].inertial.inertia.iyy, robot.links[3].inertial.inertia.izz, robot.links[3].inertial.inertia.ixy,robot.links[3].inertial.inertia.iyz, robot.links[3].inertial.inertia.ixz],
                [robot.links[4].inertial.inertia.ixx, robot.links[4].inertial.inertia.iyy, robot.links[4].inertial.inertia.izz, robot.links[4].inertial.inertia.ixy,robot.links[4].inertial.inertia.iyz, robot.links[4].inertial.inertia.ixz],
                [robot.links[5].inertial.inertia.ixx, robot.links[5].inertial.inertia.iyy, robot.links[5].inertial.inertia.izz, robot.links[5].inertial.inertia.ixy,robot.links[5].inertial.inertia.iyz, robot.links[5].inertial.inertia.ixz],
                [robot.links[6].inertial.inertia.ixx, robot.links[6].inertial.inertia.iyy, robot.links[6].inertial.inertia.izz, robot.links[6].inertial.inertia.ixy,robot.links[6].inertial.inertia.iyz, robot.links[6].inertial.inertia.ixz],
                [robot.links[7].inertial.inertia.ixx, robot.links[7].inertial.inertia.iyy, robot.links[7].inertial.inertia.izz, robot.links[7].inertial.inertia.ixy,robot.links[7].inertial.inertia.iyz, robot.links[7].inertial.inertia.ixz]
            ]

        links = []

        for j in range(6):
            link = RevoluteDH(
                d=d[j],
                a=a[j],
                alpha=alpha[j],
                m=mass[j],
                r=center_of_mass[j],
                I=inertia[j],
                G=G[j]
                # B=B[j]
            )
            links.append(link)
    
        super().__init__(
            links,
            name="robot",
            manufacturer="Robotics",
            keywords=('dynamics', 'symbolic'),
            symbolic=symbolic
        )
        # print("FFFFUCCCUCUCUCUCCUCUCCKKKKK")
        # zero angles
        self.addconfiguration("qz", np.array([0, 0, 0, 0, 0, 0]))
        # horizontal along the x-axis
        self.addconfiguration("qr", np.r_[180, 0, 0, 0, 90, 0]*deg)

        # nominal table top picking pose
        self.addconfiguration("qn", np.array([0, 0, 0, 0, 0, 0]))
        
    def return_configuration(self):
        return self.length_1, self.length_2, self.length_3
if __name__ == '__main__':    # pragma nocover

    robot = RandomRobot(symbolic=False)
    # print(robot)
    # print(robot.dynamics())
    symbolic = False
    if symbolic:
        import spatialmath.base.symbolic as sym
        zero = sym.zero()
        pi = sym.pi()
    else:
        from math import pi
        zero = 0.0
    # Set the desired end-effector pose
    # print(robot.q)
    deg = pi / 180
    q =  np.r_[180, 90, 45, 45, 90, 0]*deg
    
    print(robot.fkine(q) * sm.SE3(0, 0, 0.04))
    # robot.ikine_6s
    # robot.ikine_global
    # robot.ikine_LMS
    # robot.ikine_mmc
    T = SE3(0.7, 0.2, 0.1) * SE3.RPY([0, 1, 0])
    print(T)
    print(robot.ikine_LM(T=T))
    print(robot.ikine_LMS(T=T))

    print(robot.ikine_LM(T=robot.fkine(q) * sm.SE3(0, 0, 0.04)))
    print(robot.ikine_LMS(T=robot.fkine(q) * sm.SE3(0, 0, 0.04)))
    # print(robot.ikine_global(T=robot.fkine(q) * sm.SE3(0, 0, 0.04)))
    # print(robot.ikine_mmc(T=robot.fkine(q) * sm.SE3(0, 0, 0.04)))
    # print(robot.ikine_6s(T=robot.fkine(q) * sm.SE3(0, 0, 0.04)))
    print(robot.manipulability(q=q))
    # print(robot.manipulability(J=robot.fkine(q) * sm.SE3(0, 0, 0.04)))
