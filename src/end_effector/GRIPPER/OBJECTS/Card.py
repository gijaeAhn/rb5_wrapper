import numpy as np
from time import sleep
from GRIPPER import Gripper
from GRIPPER.Gripper import FREQUENCY
from datetime import datetime

def ScoopingCard0():#position: (130,70)
    print("   [GRIPPER/ CARD]")
    timeStep = 0.0
    firstRun = True
    tempEncoderVar = np.zeros(2)
    prevtempEncoderVar = np.zeros(2)
    encoderDifference = np.zeros(2)

    scoopingPosition = [3.96, 38.16] # new ready 323,19   258,43
    grabPosition = [-38.16, 50.76] # all new grab, if you use this point you have to set the velocity integral gain as 0.

    Gripper.SetControlState()
    Gripper.SetMotorPosition(scoopingPosition)
    sleep(1.0)
    Gripper.SetStiffness([30,30])
    Gripper.SetVelocityGain([0.1,0.1])

    while(timeStep < 5.5):
        tempEncoderVar = Gripper.GetEncoderValue()

        if(firstRun):
            prevtempEncoderVar = tempEncoderVar
            firstRun = False
        else :
            pass

        encoderDifference = abs(tempEncoderVar - prevtempEncoderVar)

        if encoderDifference[0] > 0.005 or encoderDifference[1] > 0.005:
            Gripper.SetStiffness([80,80])
            Gripper.SetVelocityGain([0.2, 0.2])
            Gripper.SetMotorPosition(grabPosition)

        else :
            pass

        timeStep += 1/FREQUENCY
        sleep(1/FREQUENCY)
        prevtempEncoderVar = tempEncoderVar

    Gripper.SetIdleState()

def Lobbing():
    print("   [GRIPPER/ CARD]")
    timeStep = 0.0
    firstRun = True
    tempEncoderVar = np.zeros(2)
    prevtempEncoderVar = np.zeros(2)
    encoderDifference = np.zeros(2)

    scoopingPosition = [139.17257309 ,58.8138485]
    grabPosition = [84.69614089 ,70.45108747]
    lobPosition = [52.0476439  ,95.67704558]

    Gripper.SetControlState()
    Gripper.SetMotorPosition(scoopingPosition)
    print("Scoop Real Position : ", Gripper.GetMotorPosition())
    Gripper.SetStiffness([20,20])
    Gripper.SetVelocityGain([0.1,0.1])

    ifGrabbed = False
    
    Encoder_1 = Gripper.GetEncoderValue()
    Encoder_2 = Gripper.GetEncoderValue()

    while(timeStep < 5 and Gripper.GetMotorPosition()[1] < 67):
        encoderDifference = abs(Encoder_1 - Encoder_2)
        if encoderDifference[0] > 0.001 or encoderDifference[1] > 0.001:
            sleep(0.5)
            Gripper.SetStiffness([30,30])
            Gripper.SetVelocityGain([0.6, 0.6])
            Gripper.SetMotorPosition(grabPosition)
            print("Grab Real Position : ", Gripper.GetMotorPosition())
            ifGrabbed =True
            break
        timeStep += 1/FREQUENCY
        sleep(1/FREQUENCY)
        Encoder_2 = Encoder_1
        Encoder_1 = Gripper.GetEncoderValue()

    if ifGrabbed :
        Gripper.SetStiffness([120,120])
        Gripper.SetVelocityGain([0.3, 0.8])
        Gripper.SetMotorPosition(lobPosition)
        print("Lob Real Position : ", Gripper.GetMotorPosition())

    sleep(0.5)
    print("Tossing End")
    # Gripper.SetIdleState()

    #pcb,card 120,130
    #motor 110, 150

def Searching():
    searchingPosition = [ 217 ,-8.56058121]

    Gripper.SetControlState()
    Gripper.SetMotorPosition(searchingPosition)
    print("Searching Real Position : ", Gripper.GetMotorPosition())
    Gripper.SetStiffness([20,20])
    Gripper.SetVelocityGain([0.1,0.1])
    sleep(0.5)


