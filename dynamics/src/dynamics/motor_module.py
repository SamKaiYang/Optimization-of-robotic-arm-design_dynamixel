#!/usr/bin/env python3
import numpy as np
import os 
import pandas as pd



class TECO_motor():
    '''
        TECO size S1, S2, M1, M2
        S1: 80mm x 89.5mm x 123mm
        S2: 90mm x 105mm x 125mm
        M1: 102mm x 89.5mm x 141mm
        M2: 130mm x 105mm x 130mm
    '''
    def __init__(self, symbolic=False):
        rated_torque = [16, 38, 60, 114] # Nm
        rated_speed = [35.0, 25.0, 22.5, 15.0] # RPM
        max_torque = [32.0, 66.0, 120.0, 198.0] # Nm
        max_output_speed = [44, 31, 42, 25] # RPM
        diameter = [80, 90, 102, 130] # mm
        height = [89.5, 105, 113.5, 145] # mm
        length = [123, 125, 141, 130] # mm
        weight = [1.3, 2.0, 2.6, 4.5] # kg
        cost = [100,200,300,400]
class dynamixel_motor():
    '''
        dynamixel_motor size 42_20, 54_100, 54_200
        42_20: 42 x 84 x 42 [mm]
        54_100: 54 x 108 x 54 [mm]
        54_200: 54 x 126 x 54 [mm]
    '''
    def __init__(self, symbolic=False):
        rated_torque = [5.1, 25.3, 44.7] # Nm
        rated_speed = [29.2, 29.2, 29.0] # RPM
        max_torque = [21.6, 66.3, 73.1] # Nm
        max_output_speed = [32, 32, 32] # RPM
        diameter = [ 42, 54, 54] # mm
        height = [84, 108, 126] # mm
        length = [42, 54, 54] # mm
        weight = [0.34, 0.74, 0.855] # kg
        cost = [200,300,400]
class Kollmorgen_motor():
    '''
        Kollmorgen size RGM14, RGM17, RGM20, RGM25
        RGM14: 79mm x 95mm x 120mm
        RGM17: 90mm x 105mm x 123mm
        RGM20: 102mm x 105mm x 127mm
        RGM25: 130mm x 105mm x 131mm
    '''
    def __init__(self, symbolic=False):
        rated_torque = [13.5, 49, 61, 118] # Nm
        rated_speed = [20.0, 20.0, 15.0, 10.0] # RPM
        max_torque = [34.0, 66.0, 102.0, 194.0] # Nm
        max_output_speed = [35, 30, 25, 20] # RPM
        diameter = [79, 90, 102, 127] # mm
        height = [95, 105, 117, 145] # mm
        length = [120, 123, 127, 131] # mm
        weight = [1.57, 2.13, 2.65, 4.43] # kg



class UR_motor():
    '''
        UR size 0, 1, 2, 3, 4
        0: 64.3208mm x 76.8158mm x 88.7783mm
        1: 77.584mm x 84.2327mm x 104.6882mm
        2: 90.8154mm x 99.8623mm x 122.9227mm
        3: 118.9042mm x 130.2477mm x 133.6598mm
        4: 150.9524mm x 161.3661mm x 177.1094mm
    '''
    def __init__(self, symbolic=False):
        # rated_torque = [16, 38, 60, 114] # Nm
        rated_speed = [60,	30,	30,	30,	20] # RPM
        max_torque = [12,	28,	56,	150,	330] # Nm
        # max_output_speed = [44, 31, 42, 25] # RPM
        diameter = [64.3208,	77.584,	90.8154,	118.9042,	150.9524] # mm
        height = [76.8158,	84.2327,	99.8623,	130.2477,	161.3661] # mm
        length = [88.7783,	104.6882,	122.9227,	133.6598,	177.1094] # mm
        weight = [0.8,	1.219,	2,	3.7,	7.1] # kg



class TM_motor():
    '''
        TM size 0, 1, 2, 3, 4
        0: 90.586mm x 94.495mm x 117.6mm
        1: 90.586mm x 94.495mm x 117.6mm
        2: 90.0038mm x 97.5295mm x 108.7092mm
        3: 121.286mm x 127.386mm x 160.0011mm
        4: 151.5741mm x 155.397mm x 201.8001mm
    '''
    def __init__(self, symbolic=False):
        rated_torque = [23,	34,	41,	118,	311] # Nm
        rated_speed = [37.5,	30,	25,	30,	20] # RPM
        max_torque = [43,	54,	54,	157,	353] # Nm
        # max_output_speed = [44, 31, 42, 25] # RPM
        diameter = [90.586,	90.586,	90.0038,	121.286,	151.5741] # mm
        height = [94.495,	94.495,	97.5295,	127.386,	155.397] # mm
        length = [117.6,	117.6,	108.7092,	160.0011,	201.8001] # mm
        weight = [1.45,	1.45,	2,	4.5,	7] # kg

class motor_data():
    def __init__(self, symbolic=False):
        self.TECO_motor_data = {"rated_torque":[16, 38, 60, 114],
                    "rated_speed":[35.0, 25.0, 22.5, 15.0],
                    "max_torque":[32.0, 66.0, 120.0, 198.0],
                    "max_output_speed":[44, 31, 42, 25],
                    "diameter":[80, 90, 102, 130],
                    "height":[89.5, 105, 113.5, 145],
                    "length":[123, 125, 141, 130],
                    "weight":[1.3, 2.0, 2.6, 4.5],
                    "cost":[100,200,300,400]
                    }
        self.TECO_member = pd.DataFrame(self.TECO_motor_data)

        self.dynamixel_motor_data = {"rated_torque":[5.1, 25.3, 44.7],
                    "rated_speed":[29.2, 29.2, 29.0],
                    "max_torque":[21.6, 66.3, 73.1],
                    "max_output_speed":[32, 32, 32],
                    "diameter":[ 42, 54, 54],
                    "height":[84, 108, 126],
                    "length":[42, 54, 54],
                    "weight":[0.34, 0.74, 0.855],
                    "cost":[200,300,400]
                    }
        self.dynamixel_member = pd.DataFrame(self.dynamixel_motor_data)
        
        self.Kollmorgen_motor_data = {"rated_torque":[13.5, 49, 61, 118],
                    "rated_speed":[20.0, 20.0, 15.0, 10.0], 
                    "max_torque":[34.0, 66.0, 102.0, 194.0],
                    "max_output_speed":[35, 30, 25, 20],
                    "diameter":[79, 90, 102, 127],
                    "height":[95, 105, 117, 145],
                    "length":[120, 123, 127, 131],
                    "weight":[1.57, 2.13, 2.65, 4.43]}
        self.Kollmorgen_member = pd.DataFrame(self.Kollmorgen_motor_data)

        self.UR_motor_data = {"rated_torque":[16, 38, 60, 114, 311],
                    "rated_speed":[60,	30,	30,	30,	20],
                    "max_torque":[12,	28,	56,	150,	330],
                    "diameter":[64.3208,	77.584,	90.8154,	118.9042,	150.9524],
                    "height":[76.8158,	84.2327,	99.8623,	130.2477,	161.3661],
                    "length":[88.7783,	104.6882,	122.9227,	133.6598,	177.1094],
                    "weight":[0.8,	1.219,	2,	3.7,	7.1]}
        self.UR_member = pd.DataFrame(self.UR_motor_data)

        self.TM_motor_data = {"rated_torque":[23,	34,	41,	118,	311],
                    "rated_speed":[37.5,	30,	25,	30,	20],
                    "max_torque":[43,	54,	54,	157,	353],
                    "diameter":[90.586,	90.586,	90.0038,	121.286,	151.5741],
                    "height":[94.495,	94.495,	97.5295,	127.386,	155.397],
                    "length":[117.6,	117.6,	108.7092,	160.0011,	201.8001],
                    "weight":[1.45,	1.45,	2,	4.5,	7]}
        self.TM_member = pd.DataFrame(self.TM_motor_data)

if __name__ == '__main__':    # pragma nocover

    motor = motor_data()
    print(motor.TECO_member.head())
    print(motor.dynamixel_member.head())
    # print(motor.TECO_member.groupby("rated_torque").mean())
    # print(motor.TECO_member.columns)
    # print(len(motor.TECO_member.columns))
    # print(motor.TECO_member.index)
    # print(len(motor.TECO_member.index))

    # print(pd.concat([motor.TECO_member, motor.Kollmorgen_member, motor.UR_member, motor.TM_member], axis=0))


    # res = motor.TECO_member.append(other=motor.Kollmorgen_member, ignore_index=True)
    # print(res)

    # res = motor.TECO_member.append([motor.Kollmorgen_member, motor.UR_member, motor.TM_member], ignore_index=True)
    # print(res)
    # print(motor.TECO_member.merge(right=motor.Kollmorgen_member, how="outer"))
    # print(robot.dynamics())
