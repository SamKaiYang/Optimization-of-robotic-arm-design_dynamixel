import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3


class TECOARM1(DHRobot):
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

    .. codeauthor:: Sheng Kai Yang
    """  # noqa

    def __init__(self, symbolic=False):

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
        # a = [0, -0.42500, -0.39225, 0, 0, 0] #m
        a = [0, -0.314, -0.284, 0, 0, 0] #m # teco
        # d = [0.089459, 0, 0, 0.10915, 0.09465, 0.0823] #m
        d = [0.1301, 0, 0, 0.1145, 0.090, 0.048] #m # teco 

        alpha = [pi/2, zero, zero, pi/2, -pi/2, zero]

        # mass data, no inertia available
        # mass = [3.7000, 8.3930, 2.33, 1.2190, 1.2190, 0.1897]
        mass = [0.706627496414257, 1.81432701922486, 0.44267362700812, 0.240053004221894, 0.22616916171284, 0.3927]
        
        G= [-80,-80,-80,-50,-50,-50]   # gear ratio
        # <xacro:property name="shoulder_cog" value="0.0 0.00193 -0.02561" />
        # <xacro:property name="upper_arm_cog" value="0.0 -0.024201 0.2125" />
        # <xacro:property name="forearm_cog" value="0.0 0.0265 0.11993" />
        # <xacro:property name="wrist_1_cog" value="0.0 0.110949 0.01634" />
        # <xacro:property name="wrist_2_cog" value="0.0 0.0018 0.11099" />
        # <xacro:property name="wrist_3_cog" value="0.0 0.001159 0.0" />

        # xyz="-1.55579201081481E-05 0.00265005484815443 -0.00640979059142413"
        # xyz="4.90637956589368E-11 0.205571973027702 -0.00335989805856342"
        # xyz="-9.75479428030767E-05 0.271025707847572 0.111573843205116"
        # xyz="-0.000181761828397664 0.00219045749084071 -0.000800397394362884"
        # xyz="-0.000192919655058627 -0.00232492307126431 0.00352418959262345"
        # xyz="-4.4856E-13 0 0.025"
        # center_of_mass = [
        #         [0,     -0.02561,  0.00193],
        #         [0.2125, 0,        0.11336],
        #         [0.15,   0,        0.0265],
        #         [0,     -0.0018,   0.01634],
        #         [0,     -0.0018,   0.01634],
        #         [0,      0,       -0.001159]
        #     ]
        center_of_mass = [
                [-1.55579201081481E-05, 0.00265005484815443, -0.00640979059142413],
                [4.90637956589368E-11, 0.205571973027702, -0.003359898058563426],
                [-9.75479428030767E-05, 0.271025707847572, 0.111573843205116],
                [-0.000181761828397664, 0.00219045749084071, -0.000800397394362884],
                [-0.000192919655058627, -0.00232492307126431, 0.00352418959262345],
                [-4.4856E-13 ,0 ,0.025]
            ]
        links = []

        for j in range(6):
            link = RevoluteDH(
                d=d[j],
                a=a[j],
                alpha=alpha[j],
                m=mass[j],
                r=center_of_mass[j],
                G=G[j]
            )
            links.append(link)
    
        super().__init__(
            links,
            name="teco",
            manufacturer="teco Robotics",
            keywords=('dynamics', 'symbolic'),
            symbolic=symbolic
        )
    
        # zero angles
        self.addconfiguration("qz", np.array([0, 0, 0, 0, 0, 0]))
        # horizontal along the x-axis
        self.addconfiguration("qr", np.r_[180, 0, 0, 0, 90, 0]*deg)

        # nominal table top picking pose
        self.addconfiguration("qn", np.array([0, 0, 0, 0, 0, 0]))
if __name__ == '__main__':    # pragma nocover

    teco = TECOARM1(symbolic=False)
    print(teco)
    print(teco.dynamics())