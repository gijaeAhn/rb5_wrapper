import sys
import os
import platform
import threading
import queue
from time import sleep

#------------------------------------------------------------------------------------------------------
def setup_path():
    home_dir = os.path.expanduser('~')
    
    if platform.system() == 'Darwin':  # macOS
        sys.path.append(os.path.join(home_dir, 'Desktop/tossing/src/tossing/src/end_effector'))
    else: 
        sys.path.append(os.path.join(home_dir, 'Desktop/catkin_ws/src/tossing/src/end_effector'))
setup_path()
#------------------------------------------------------------------------------------------------------
from config import eeConfig
from lobbing import Lobbing
import GRIPPER.Gripper
from GRIPPER import Gripper
from GRIPPER import mainGripper
#------------------------------------------------------------------------------------------------------


class eeController:
    def __init__(self):
        self.running = True
        self.gripper_thread = None
        self.input_queue = queue.Queue()  # Queue to handle user input
        # eeConfig.load_config() 

    def gripper_thread_function(self):
        while self.running:
            try:
                Lobbing()
            except Exception as e:
                print(f"Error in gripper thread: {e}")
                break

    def handle_keyboard_input(self):
        while self.running:
            try:
                sleep(100)
            except EOFError:
                self.running = False
                break

    def start(self):
        try:
            self.gripper_thread = threading.Thread(
                target=self.gripper_thread_function,
                name="GripperThread"
            )
            self.gripper_thread.daemon = True
            self.gripper_thread.start()

            self.gripper_thread.join()

        except KeyboardInterrupt:
            print("\n Process Terminated.")
        except Exception as e:
            print(f"Error : {e}")
        finally:
            self.running = False
            print("Process Terminated.")

if __name__ == "__main__":
    controller = eeController()
    controller.start()
