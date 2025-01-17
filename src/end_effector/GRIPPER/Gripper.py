import sys
import os
current_dir_gripper = os.path.dirname(os.path.abspath(__file__)) # 예를들어 부모 디렉토리를 만든다면 parent_dir = os.path.join(current_dir, '..') 이렇게도 가능
sys.path.append(current_dir_gripper)
import numpy as np
import odrive
from Actuator import *

FREQUENCY = 50 #Hz

SN0 = '394434613331'
SN1 = '3868345A3539'

odrv0 = odrive.find_any(serial_number=SN0)
odrv1 = odrive.find_any(serial_number=SN1)

F0 = Actuator(odrv0, 0, 1, 45)  #0.116
F1 = Actuator(odrv1, 0, 1, 45)  #0.463

sharedTimeList = []
sharedPositionList = []

sharedData = 0

def ClearErrors():
    F0.clearErrors()
    F1.clearErrors()

def GetMotorPosition():
    tempArray = np.zeros(2)
    tempArray[0] = F0.motor_pos
    tempArray[1] = F1.motor_pos
    return tempArray

def GetStiffness():
    tempArray = np.zeros(2)
    tempArray[0] = F0.stiffness
    tempArray[1] = F1.stiffness
    return tempArray

def GetVelocityGains():
    tempArray = np.zeros(2)
    tempArray[0] = F0.vel_gain
    tempArray[1] = F1.vel_gain
    return tempArray

def GetCurrent():
    tempArray = np.zeros(2)
    tempArray[0] = F0.iBusValue()
    tempArray[1] = F1.iBusValue()
    return tempArray

def GetEncoderValue():
    tempArray = np.zeros(2)
    tempArray[0] = F0.encoder
    tempArray[1] = F1.encoder
    return tempArray

def SetMotorPosition(motorAngles):
    F0.motor_pos = motorAngles[0]
    F1.motor_pos = motorAngles[1]

# stiffness = position gain
def SetStiffness(stiffness):
    F0.stiffness = stiffness[0]
    F1.stiffness = stiffness[1]

def SetVelocityGain(velGains):
    F0.vel_gain = velGains[0]
    F1.vel_gain = velGains[1]

def SetControlState():
    ClearErrors()
    F0.armed = True
    F1.armed = True
    SetStiffness([30, 30])
    SetVelocityGain([0.3, 0.3])
    print("      [GRIPPER/ CONTROL STATE]")

def SetIdleState():
    ClearErrors()
    F0.armed = False
    F1.armed = False
    print("      [GRIPPER/ IDLE STATE]")


