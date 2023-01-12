from spatialmath import SE3
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
from math import pi
import numpy as np
from urdf_parser_py.urdf import URDF

class EPSONG3(DHRobot):
    """
    Class that models a Adept Cobra 600 SCARA manipulator

    ``Cobra600()`` is a class which models an Adept Cobra 600 SCARA robot and
    describes its kinematic characteristics using standard DH
    conventions.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.DH.Cobra600()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration

    .. note::
        - SI units are used.
        - Robot has only 4 DoF.

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
        robot = URDF.from_xml_file(os.path.dirname(os.path.realpath(__file__))+"/urdf"+"/scara_new.urdf")

        L = [RevoluteDH(d=0.387, a=0.325, qlim=[-50*deg, 50*deg]),
                RevoluteDH(a=0.275, alpha=pi, qlim=[-88*deg, 88*deg]),
                PrismaticDH(qlim=[0, 0.210]),
                RevoluteDH()]
        
        # robot length values (metres)
        a = [0, -0.42500, -0.39225, 0, 0, 0]
        d = [0.089459, 0, 0, 0.10915, 0.09465, 0.0823]

        alpha = [pi/2, zero, zero, pi/2, -pi/2, zero]

        # mass data, no inertia available
        mass = [2, 0.5, 0.5, 0.3] 
        center_of_mass = [
                [0, 0, -0.129],
                [-0.120, 0, -0.129],
                [-0.250, 0, -0.129]
            ]
        links = []

        for j in range(4):
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
            name="epson g3",
            manufacturer="epson",
            keywords=('dynamics', 'symbolic'),
            symbolic=symbolic
        )
    
        # zero angles
        self.addconfiguration("qz", np.array([0, 0, 0, 0, 0, 0]))
        # horizontal along the x-axis
        self.addconfiguration("qr", np.r_[180, 0, 0, 0, 90, 0]*deg)

        # nominal table top picking pose
        self.addconfiguration("qn", np.array([0, 0, 0, 0, 0, 0]))

        
if __name__ == '__main__':   # pragma nocover
    epson = EPSONG3()
    epson.plot(q=epson.qz)
    print(epson)