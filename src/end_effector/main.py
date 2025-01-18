import sys
import os

home_dir = os.path.expanduser('~')
sys.path.append(os.path.join(home_dir, 'Desktop/catkin_ws/src/tossing/src/end_effector'))

import threading
import GRIPPER.Gripper
from GRIPPER import Gripper
from GRIPPER import mainGripper

from lobbing import Lobbing


def threadGripper():
    while(True):
        # print("[GRIPPER THREAD]")
        # mainGripper.mainGripper()
        Lobbing()

if __name__ == "__main__":
    gripperThread = threading.Thread(target=threadGripper)

    gripperThread.start()
    gripperThread.join()

    print("Program is terminated.")
