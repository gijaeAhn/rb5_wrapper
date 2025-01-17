import sys
import os
current_dir_gripper = os.path.dirname(os.path.abspath(__file__)) # 예를들어 부모 디렉토리를 만든다면 parent_dir = os.path.join(current_dir, '..') 이렇게도 가능
sys.path.append(current_dir_gripper)
import OBJECTS.Card
import TestMotion


######## select control signal ########
controlSignal = 'card'
# controlSignal = 'testMotion'
######################################

def switchCase(case):
    if case == 'card':
        # OBJECTS.Card.ScoopingCard0()
        OBJECTS.Card.Lobbing()


    elif case == 'testMotion':
        # TestMotion.TestGetEncoder()
        TestMotion.TestMotion()
        # TestMotion.TestMotionStop()
        pass

    else:
        print("Check the controlSignal")

# Gripper main is started
def mainGripper():
    switchCase(controlSignal)
