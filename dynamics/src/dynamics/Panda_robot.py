import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
from urdf_parser_py.urdf import URDF

class Panda(DHRobot):
    """
    A class representing the Panda robot arm.

    ``Panda()`` is a class which models a Franka-Emika Panda robot and
    describes its kinematic characteristics using modified DH
    conventions.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.DH.Panda()
        >>> print(robot)

    .. note::
        - SI units of metres are used.
        - The model includes a tool offset.

    :references:
        - https://frankaemika.github.io/docs/control_parameters.html

    .. codeauthor:: Samuel Drew
    .. codeauthor:: Peter Corke
    """

    def __init__(self):
        robot = URDF.from_xml_file(os.path.dirname(os.path.realpath(__file__))+"/urdf"+"/panda.urdf")

        # deg = np.pi/180
        mm = 1e-3
        tool_offset = (103)*mm

        flange = (107)*mm
        # d7 = (58.4)*mm

        # This Panda model is defined using modified
        # Denavit-Hartenberg parameters
        L = [
            RevoluteDH(
                a=0.0,
                d=0.333,
                alpha=0.0,
                qlim=np.array([-2.8973, 2.8973])
            ),

            RevoluteDH(
                a=0.0,
                d=0.0,
                alpha=-np.pi/2,
                qlim=np.array([-1.7628, 1.7628])
            ),

            RevoluteDH(
                a=0.0,
                d=0.316,
                alpha=np.pi/2,
                qlim=np.array([-2.8973, 2.8973])
            ),

            RevoluteDH(
                a=0.0825,
                d=0.0,
                alpha=np.pi/2,
                qlim=np.array([-3.0718, -0.0698])
            ),

            RevoluteDH(
                a=-0.0825,
                d=0.384,
                alpha=-np.pi/2,
                qlim=np.array([-2.8973, 2.8973])
            ),

            RevoluteDH(
                a=0.0,
                d=0.0,
                alpha=np.pi/2,
                qlim=np.array([-0.0175, 3.7525])
            ),

            RevoluteDH(
                a=0.088,
                d=flange,
                alpha=np.pi/2,
                qlim=np.array([-2.8973, 2.8973])
            )
        ]

        # tool = transl(0, 0, tool_offset) @  trotz(-np.pi/4)

        
        a = [0, -robot.joints[3].origin.xyz[1], -robot.joints[4].origin.xyz[1], 0, 0, 0]
        # a = [0, -j3.y, -j4.y, 0, 0, 0]
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
            name='Panda',
            manufacturer='Franka Emika',
            meshdir='meshes/FRANKA-EMIKA/Panda',
            )

        self.addconfiguration("qz", [0, 0, 0, 0, 0, 0, 0])
        self.addconfiguration("qr", np.r_[0, -0.3, 0, -2.2, 0, 2.0, np.pi/4])


if __name__ == '__main__':   # pragma nocover

    panda = Panda()
    panda.plot(panda.qz, dt=10)
    print(panda)