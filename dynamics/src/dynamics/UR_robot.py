import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3

class UR3(DHRobot):
    
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


        self.length_1 = 0.24365
        self.length_2 = 0.21325
        self.length_3 = 0.09465

        # robot length values (metres)
        a = [0, -0.24365, -0.21325, 0, 0, 0]
        d = [0.1519, 0, 0, 0.11235, 0.08535, 0.0819]

        alpha = [pi/2, zero, zero, pi/2, -pi/2, zero]

        # mass data, no inertia available
        mass = [2, 3.42, 1.26, 0.8, 0.8, 0.35]
        center_of_mass = [
                	[0, -0.02, 0],
                	[0.13, 0, 0.1157],
                [0.05, 0, 0.0238],
                [0, 0, 0.01],
                [0, 0, 0.01],
                [0, 0, -0.02]
            ]
        links = []

        for j in range(6):
            link = RevoluteDH(
                d=d[j],
                a=a[j],
                alpha=alpha[j],
                m=mass[j],
                r=center_of_mass[j],
                G=1
            )
            links.append(link)
    
        super().__init__(
            links,
            name="UR5",
            manufacturer="Universal Robotics",
            keywords=('dynamics', 'symbolic'),
            symbolic=symbolic
        )
    
        # zero angles
        self.addconfiguration("qz", np.array([0, 0, 0, 0, 0, 0]))
        # horizontal along the x-axis
        self.addconfiguration("qr", np.r_[180, 0, 0, 0, 90, 0]*deg)

        # nominal table top picking pose
        self.addconfiguration("qn", np.array([0, 0, 0, 0, 0, 0]))
    def return_configuration(self):
        return self.length_1, self.length_2, self.length_3
class UR5(DHRobot):
    """
    Class that models a Universal Robotics UR5 manipulator

    :param symbolic: use symbolic constants
    :type symbolic: bool
    
    Defined joint configurations are:

    - qz, zero joint angle configuration
    - qr, arm horizontal along x-axis

    .. note::
        - SI units are used.
    .. codeauthor:: Peter Corke
    """ 

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


        self.length_1 = 0.42500
        self.length_2 = 0.39225
        self.length_3 = 0.09465

        # robot length values (metres)
        a = [0, -0.42500, -0.39225, 0, 0, 0]
        d = [0.089459, 0, 0, 0.10915, 0.09465, 0.0823]

        alpha = [pi/2, zero, zero, pi/2, -pi/2, zero]

        # mass data, no inertia available
        mass = [3.7000, 8.3930, 2.33, 1.2190, 1.2190, 0.1897]
        center_of_mass = [
                [0,     -0.02561,  0.00193],
                [0.2125, 0,        0.11336],
                [0.15,   0,        0.0265],
                [0,     -0.0018,   0.01634],
                [0,     -0.0018,   0.01634],
                [0,      0,       -0.001159]
            ]
        links = []

        for j in range(6):
            link = RevoluteDH(
                d=d[j],
                a=a[j],
                alpha=alpha[j],
                m=mass[j],
                r=center_of_mass[j],
                G=1
            )
            links.append(link)
    
        super().__init__(
            links,
            name="UR5",
            manufacturer="Universal Robotics",
            keywords=('dynamics', 'symbolic'),
            symbolic=symbolic
        )
    
        # zero angles
        self.addconfiguration("qz", np.array([0, 0, 0, 0, 0, 0]))
        # horizontal along the x-axis
        self.addconfiguration("qr", np.r_[180, 0, 0, 0, 90, 0]*deg)

        # nominal table top picking pose
        self.addconfiguration("qn", np.array([0, 0, 0, 0, 0, 0]))
    def return_configuration(self):
        return self.length_1, self.length_2, self.length_3
class UR10(DHRobot):
    """
    Class that models a Universal Robotics UR10 manipulator

    :param symbolic: use symbolic constants
    :type symbolic: bool

    Defined joint configurations are:

    - qz, zero joint angle configuration
    - qr, arm horizontal along x-axis

    .. note::
        - SI units are used.

    .. codeauthor:: Peter Corke
    """

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
        a = [0, -0.612, -0.5723, 0, 0, 0]
        d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922]

        alpha = [pi/2, zero, zero, pi/2, -pi/2, zero]

        # mass data, no inertia available
        mass = [7.1, 12.7, 4.27, 2.000, 2.000, 0.365]
        center_of_mass = [
                [0.021, 0,     0.027],
                [0.38,  0,     0.158],
                [0.24,  0,     0.068],
                [0.0,   0.007, 0.018],
                [0.0,   0.007, 0.018],
                [0,     0,     -0.026]
            ]
        links = []

        for j in range(6):
            link = RevoluteDH(
                d=d[j],
                a=a[j],
                alpha=alpha[j],
                m=mass[j],
                r=center_of_mass[j],
                G=1
            )
            links.append(link)
    
        super().__init__(
            links,
            name="UR10",
            manufacturer="Universal Robotics",
            keywords=('dynamics', 'symbolic'),
            symbolic=symbolic
        )
    
        # zero angles
        self.addconfiguration("qz", np.array([0, 0, 0, 0, 0, 0]))
        # horizontal along the x-axis
        self.addconfiguration("qr", np.r_[180, 0, 0, 0, 90, 0]*deg)
        # nominal table top picking pose
        self.addconfiguration("qn", np.array([0, 0, 0, 0, 0, 0]))
    def return_configuration(self):
        return self.length_1, self.length_2, self.length_3
class UR10e(DHRobot):
    """
    Class that models a Universal Robotics UR10e manipulator

    :param symbolic: use symbolic constants
    :type symbolic: bool

    Defined joint configurations are:

    - qz, zero joint angle configuration
    - qr, arm horizontal along x-axis

    .. note::
        - SI units are used.

    .. codeauthor:: Yi He Yang
    """

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
        a = [0, -0.6127, -0.57155, 0, 0, 0]
        d = [0.1807, 0, 0, 0.17415, 0.11985, 0.11655]

        alpha = [pi/2, zero, zero, pi/2, -pi/2, zero]

        # mass data, no inertia available
        mass = [7.369, 13.051, 3.989, 2.1, 1.98, 0.615]
        center_of_mass = [
                [0.021, 0.000, 0.027],
                [0.38, 0.000, 0.158],
                [0.24, 0.000, 0.068],
                [0.000, 0.007, 0.018],
                [0.000, 0.007, 0.018],
                [0, 0, -0.026]
            ]
        links = []

        for j in range(6):
            link = RevoluteDH(
                d=d[j],
                a=a[j],
                alpha=alpha[j],
                m=mass[j],
                r=center_of_mass[j],
                G=1
            )
            links.append(link)
    
        super().__init__(
            links,
            name="UR10e",
            manufacturer="Universal Robotics",
            keywords=('dynamics', 'symbolic'),
            symbolic=symbolic
        )
    
        # zero angles
        self.addconfiguration("qz", np.array([0, 0, 0, 0, 0, 0]))
        # horizontal along the x-axis
        self.addconfiguration("qr", np.r_[180, 0, 0, 0, 90, 0]*deg)

        # nominal table top picking pose
        self.addconfiguration("qn", np.array([0, 0, 0, 0, 0, 0]))
    def return_configuration(self):
        return self.length_1, self.length_2, self.length_3

class UR16e(DHRobot):
    """
    Class that models a Universal Robotics UR16e manipulator

    :param symbolic: use symbolic constants
    :type symbolic: bool

    Defined joint configurations are:

    - qz, zero joint angle configuration
    - qr, arm horizontal along x-axis

    .. note::
        - SI units are used.

    .. codeauthor:: Yi He Yang
    """

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
        a = [0, -0.4784, -0.36, 0, 0, 0]
        d = [0.1807, 0, 0, 0.17415, 0.11985, 0.11655]

        alpha = [pi/2, zero, zero, pi/2, -pi/2, zero]

        # mass data, no inertia available
        mass = [7.369, 10.45, 4.321, 2.180, 2.033, 0.907]
        center_of_mass = [
                [0.000, -0.016, 0.030],
                [0.302, 0.000, 0.160],
                [0.194, 0.000, 0.065],
                [0.000, -0.009, 0.011],
                [0.000, 0.018, 0.012],
                [0, 0, -0.044]
            ]
        links = []
        G= [-80,-80,-80,-50,-50,-50]   # gear ratio
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
            name="UR16e",
            manufacturer="Universal Robotics",
            keywords=('dynamics', 'symbolic'),
            symbolic=symbolic
        )
    
        # zero angles
        self.addconfiguration("qz", np.array([0, 0, 0, 0, 0, 0]))
        # horizontal along the x-axis
        self.addconfiguration("qr", np.r_[180, 0, 0, 0, 90, 0]*deg)

        # nominal table top picking pose
        self.addconfiguration("qn", np.array([0, 0, 0, 0, 0, 0]))
    def return_configuration(self):
        return self.length_1, self.length_2, self.length_3

if __name__ == '__main__':    # pragma nocover

    ur = UR5(symbolic=False)
    print(ur)
    # print(ur5.dyntable())
