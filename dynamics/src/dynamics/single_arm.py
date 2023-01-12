#!/usr/bin/env python3
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
import matplotlib.pyplot as plt
from spatialmath import SE3
from urdf_parser_py.urdf import URDF
import os


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

        links = []

        for j in range(7):
            link = RevoluteDH(
                d=d[j],
                alpha=alpha[j],
                # theta=theta[j],
                a=a[j],
                m=mass[j],
                r=center_of_mass[j],
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
    # # TODO: fixed DH table
    # from math import pi
    # deg = pi / 180
    # qz = np.array([90, -90, 0, 45, 0, 45, 90])*deg
    # robot.plot(qz, dt = 10)
    