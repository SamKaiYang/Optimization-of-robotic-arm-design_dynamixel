"""
@author: Gautam Sinha, Indian Institute of Technology, Kanpur
  (original MATLAB version)
@author: Peter Corke
@author: Samuel Drew
"""

from roboticstoolbox import DHRobot, RevoluteDH
from math import pi
import numpy as np


class denso(DHRobot):
    """
    Class that models a Kuka KR5 manipulator

    ``KR5()`` is a class which models a Kuka KR5 robot and
    describes its kinematic characteristics using standard DH
    conventions.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.DH.KR5()
        >>> print(robot)

    Defined joint configurations are:

      - qk1, nominal working position 1
      - qk2, nominal working position 2
      - qk3, nominal working position 3

    .. note::
      - SI units of metres are used.
      - Includes an 11.5cm tool in the z-direction

    :references:
      - https://github.com/4rtur1t0/ARTE/blob/master/robots/KUKA/KR5_arc/parameters.m

    .. codeauthor:: Gautam Sinha, Indian Institute of Technology, Kanpur (original MATLAB version)
    .. codeauthor:: Samuel Drew
    .. codeauthor:: Peter Corke
    """  # noqa

    def __init__(self):
        deg = pi / 180

        # mass data, no inertia available
        mass = [4.327, 9.475, 4.097, 1.779, 1.2190, 0.1897]
        center_of_mass = [
                [0.013885, -0.005997, 0.043991],
                [0.015837, -0.073696, 0.010087],
                [0.011544, -0.057408, 0.006203],
                [0.003998, -0.004975, 0.052919],
                [0.011902, -0.004739, 0.010398],
                [-0.000401015, 0.0770342, 0.0962886]
            ]
        inertia = [
                [0.025492, 0, 0, 0.032502, -3.26672e-18, 0.01779 ],
                [0.108024,0,0,0.025579,1.93827e-17,0.112871 ],
                [0.015999,0,0,0.00638,2.33191e-18,0.016882 ],
                [0.005106,0,0,0.005266,-5.87308e-19,0.002621],
                [0.111172755531,0.0,0.0,0.111172755531,0.0,0.21942],
                [0.0171364731454,0.0,0.0,0.0171364731454,0.0,0.033822]
            ]
        # Updated values form ARTE git. Old values left as comments

        L1 = RevoluteDH(
            a=0, d=0.125, alpha=pi / 2, qlim=[-160 * deg, 160 * deg], 
            m = mass[0], 
            r = center_of_mass[0],
            I=inertia[0] # alpha=pi / 2,
        )
        L2 = RevoluteDH(
            a=0.21, d=0, alpha=0, qlim=[-30 * deg, 210 * deg], 
            m = mass[1], 
            r = center_of_mass[1],
            I=inertia[1] # d=0.135,  # alpha=pi,
        )
        L3 = RevoluteDH(
            a=-0.075,
            d=0,  # d=0.135,
            alpha=-pi / 2,  # alpha=-pi / 2,
            qlim=[90 * deg, 270 * deg],
            m = mass[2],
            r = center_of_mass[2],
            I=inertia[2]
        )
        L4 = RevoluteDH(
            a=0.0,
            d=0.21,  # d=0.62,
            alpha=pi / 2,  # alpha=pi / 2,
            qlim=[-160 * deg, 160 * deg],
            m = mass[3],
            r = center_of_mass[3],
            I=inertia[3]
        )
        L5 = RevoluteDH(
            a=0.0, d=0.0, alpha=-pi / 2, qlim=[-120 * deg, 120 * deg],
            m = mass[4],
            r = center_of_mass[4],
            I=inertia[4] # alpha=-pi / 2,
        )
        L6 = RevoluteDH(a=0, d=0.07, alpha = 0, qlim=[-360 * deg, 360 * deg],
                        m = mass[5],
                        r = center_of_mass[5],
                        I=inertia[5])

        L = [L1, L2, L3, L4, L5, L6]

        # Create SerialLink object
        super().__init__(
            L,
            # meshdir="KUKA/KR5_arc",
            name="denso",
            manufacturer="denso",
            # meshdir="meshes/KUKA/KR5_arc",
        )

        self.qr = np.array([pi / 4, pi / 3, pi / 4, pi / 6, pi / 4, pi / 6])
        self.qz = np.zeros(6)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)

        # self.addconfiguration_attr(
        #     "qk1", np.array([pi / 4, pi / 3, pi / 4, pi / 6, pi / 4, pi / 6])
        # )
        # self.addconfiguration_attr(
        #     "qk2", np.array([pi / 4, pi / 3, pi / 6, pi / 3, pi / 4, pi / 6])
        # )
        # self.addconfiguration_attr(
        #     "qk3", np.array([pi / 6, pi / 3, pi / 6, pi / 3, pi / 6, pi / 3])
        # )


if __name__ == "__main__":  # pragma nocover
    robot = denso()
    print(robot)
    robot.teach()