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
            Gripper.SetStiffness([120,120])
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

    scoopingPosition = [151.6651547 ,  37.69243151]
    grabPosition = [100.20967841,  40.26199579]
    # lobPosition = [78.66342366, 58.34278286]
    lobPosition = [64.02540743, 76.77497149]

    Gripper.SetControlState()
    Gripper.SetMotorPosition(scoopingPosition)
    sleep(1.0)
    Gripper.SetStiffness([20,20])
    Gripper.SetVelocityGain([0.1,0.1])

    runOnce = True

    while(timeStep < 5):

        while(runOnce and timeStep < 7 and Gripper.GetMotorPosition()[1] < 39):
            tempEncoderVar = Gripper.GetEncoderValue()

            if(firstRun):
                prevtempEncoderVar = tempEncoderVar
                firstRun = False
            else :
                pass

            encoderDifference = abs(tempEncoderVar - prevtempEncoderVar)

            if encoderDifference[0] > 0.005 or encoderDifference[1] > 0.005:
                Gripper.SetStiffness([80,80])
                Gripper.SetVelocityGain([0.8, 0.8])
                Gripper.SetMotorPosition(grabPosition)

            else :
                pass

            timeStep += 1/FREQUENCY
            sleep(1/FREQUENCY)
            prevtempEncoderVar = tempEncoderVar

        runOnce = False

        Gripper.SetVelocityGain([0.6, 0.6])
        Gripper.SetMotorPosition(lobPosition)

        timeStep += 1/FREQUENCY
        sleep(1/FREQUENCY)
    

    Gripper.SetIdleState()

    #pcb,card 120,130
    #motor 110, 150