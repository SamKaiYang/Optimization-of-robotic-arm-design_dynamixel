import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3

# TODO:fixed TM robot data 
class TM5_700(DHRobot):
    """
    Class that models a Techman Robot TM5 700 manipulator

    :param symbolic: use symbolic constants
    :type symbolic: bool

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

        self.length_1 = 0.3290
        self.length_2 = 0.3115
        self.length_3 = 0.1060
        # robot length values (metres)
        a = [0, -0.3290, -0.3115, 0, 0, 0]
        d = [0.1452, 0, 0, 0.1223, 0.1060, 0.11315]

        alpha = [pi/2, zero, zero, pi/2, -pi/2, zero]

        # mass data, no inertia available
        mass = [4.5, 11.0, 2.5, 1.45, 1.45, 0.4]
        radius = [0.06, 0.06, 0.045, 0.045, 0.0045, 0.0045]
        length = [0.15, 0.3290, 0.3115, 0.06, 0.12, 0.12]
        center_of_mass = [
                [(0.0833333 * mass[0] * (3 * radius[0] * radius[0] + length[0] * length[0])),     (0.5 * mass[0] * radius[0] * radius[0]),  (0.0833333 * mass[0] * (3 * radius[0] * radius[0] + length[0] * length[0]))], # cylinder_inertial_y radius="0.06" length="0.15" mass="${shoulder_mass}">
                [(0.0833333 * mass[1] * (3 * radius[1] * radius[1] + length[1] * length[1])),     (0.5 * mass[1] * radius[1] * radius[1]),  (0.0833333 * mass[1] * (3 * radius[1] * radius[1] + length[1] * length[1]))], # cylinder_inertial_y radius="0.06" length="${arm_1_length}" mass="${arm_1_mass}">
                [(0.0833333 * mass[2] * (3 * radius[2] * radius[2] + length[2] * length[2])),     (0.5 * mass[2] * radius[2] * radius[2]),  (0.0833333 * mass[2] * (3 * radius[2] * radius[2] + length[2] * length[2]))], # cylinder_inertial_y radius="0.045" length="${arm_2_length}" mass="${arm_2_mass}">
                [(0.0833333 * mass[3] * (3 * radius[3] * radius[3] + length[3] * length[3])),     (0.5 * mass[3] * radius[3] * radius[3]),  (0.0833333 * mass[3] * (3 * radius[3] * radius[3] + length[3] * length[3]))], # cylinder_inertial_y radius="0.045" length="0.06" mass="${wrist_1_mass}">
                [(0.0833333 * mass[4] * (3 * radius[4] * radius[4] + length[4] * length[4])),     (0.0833333 * mass[4] * (3 * radius[4] * radius[4] + length[4] * length[4])),   (0.5 * mass[4] * radius[4] * radius[4])], # cylinder_inertial_z radius="0.045" length="0.12" mass="${wrist_2_mass}">
                [(0.0833333 * mass[5] * (3 * radius[5] * radius[5] + length[5] * length[5])),     (0.0833333 * mass[5] * (3 * radius[5] * radius[5] + length[5] * length[5])),   (0.5 * mass[5] * radius[5] * radius[5])] # cylinder_inertial_z radius="0.045" length="0.12" mass="${wrist_3_mass}">
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
            name="TM5_700",
            manufacturer="Techman Robot",
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
class TM5_900(DHRobot):
    """
    Class that models a Techman Robot TM5 900 manipulator

    :param symbolic: use symbolic constants
    :type symbolic: bool

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
        self.length_1 = 0.4290
        self.length_2 = 0.4115
        self.length_3 = 0.1060
        # robot length values (metres)
        a = [0, -0.4290, -0.4115, 0, 0, 0]
        d = [0.1452, 0, 0, 0.1223, 0.1060, 0.11315]

        alpha = [pi/2, zero, zero, pi/2, -pi/2, zero]

        # mass data, no inertia available
        mass = [4.5, 11.0, 2.5, 1.45, 1.45, 0.4]
        radius = [0.06, 0.06, 0.045, 0.045, 0.0045, 0.0045]
        length = [0.15, 0.4290, 0.4115, 0.06, 0.12, 0.12]
        center_of_mass = [
                [(0.0833333 * mass[0] * (3 * radius[0] * radius[0] + length[0] * length[0])),     (0.5 * mass[0] * radius[0] * radius[0]),  (0.0833333 * mass[0] * (3 * radius[0] * radius[0] + length[0] * length[0]))], # cylinder_inertial_y radius="0.06" length="0.15" mass="${shoulder_mass}">
                [(0.0833333 * mass[1] * (3 * radius[1] * radius[1] + length[1] * length[1])),     (0.5 * mass[1] * radius[1] * radius[1]),  (0.0833333 * mass[1] * (3 * radius[1] * radius[1] + length[1] * length[1]))], # cylinder_inertial_y radius="0.06" length="${arm_1_length}" mass="${arm_1_mass}">
                [(0.0833333 * mass[2] * (3 * radius[2] * radius[2] + length[2] * length[2])),     (0.5 * mass[2] * radius[2] * radius[2]),  (0.0833333 * mass[2] * (3 * radius[2] * radius[2] + length[2] * length[2]))], # cylinder_inertial_y radius="0.045" length="${arm_2_length}" mass="${arm_2_mass}">
                [(0.0833333 * mass[3] * (3 * radius[3] * radius[3] + length[3] * length[3])),     (0.5 * mass[3] * radius[3] * radius[3]),  (0.0833333 * mass[3] * (3 * radius[3] * radius[3] + length[3] * length[3]))], # cylinder_inertial_y radius="0.045" length="0.06" mass="${wrist_1_mass}">
                [(0.0833333 * mass[4] * (3 * radius[4] * radius[4] + length[4] * length[4])),     (0.0833333 * mass[4] * (3 * radius[4] * radius[4] + length[4] * length[4])),   (0.5 * mass[4] * radius[4] * radius[4])], # cylinder_inertial_z radius="0.045" length="0.12" mass="${wrist_2_mass}">
                [(0.0833333 * mass[5] * (3 * radius[5] * radius[5] + length[5] * length[5])),     (0.0833333 * mass[5] * (3 * radius[5] * radius[5] + length[5] * length[5])),   (0.5 * mass[5] * radius[5] * radius[5])] # cylinder_inertial_z radius="0.045" length="0.12" mass="${wrist_3_mass}">
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
            name="TM5_900",
            manufacturer="Techman Robot",
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
class TM12(DHRobot):
    """
    Class that models a Techman Robot TM12 manipulator

    :param symbolic: use symbolic constants
    :type symbolic: bool

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

        self.length_1 = 0.4290
        self.length_2 = 0.4115
        self.length_3 = 0.1060
        # robot length values (metres)
        a = [0, -0.3290, -0.3115, 0, 0, 0]
        d = [0.1452, 0, 0, 0.1223, 0.1060, 0.11315]

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
            name="TM12",
            manufacturer="Techman Robot",
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
class TM14(DHRobot):
    """
    Class that models a Techman Robot TM14 manipulator

    :param symbolic: use symbolic constants
    :type symbolic: bool

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

        self.length_1 = 0.4290
        self.length_2 = 0.4115
        self.length_3 = 0.1060
        # robot length values (metres)
        a = [0, -0.3290, -0.3115, 0, 0, 0]
        d = [0.1452, 0, 0, 0.1223, 0.1060, 0.11315]

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
            name="TM14",
            manufacturer="Techman Robot",
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

    TM5 = TM5_700(symbolic=False)
    print(TM5_700)
    # print(TM5.dyntable())
