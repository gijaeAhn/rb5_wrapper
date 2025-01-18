import sys
import os
import platform
def setup_path():
    home_dir = os.path.expanduser('~')
    
    if platform.system() == 'Darwin':  # macOS
        sys.path.append(os.path.join(home_dir, 'Desktop/tossing/src/tossing/src/end_effector'))
    else: 
        sys.path.append(os.path.join(home_dir, 'Desktop/catkin_ws/src/tossing/src/end_effector'))
setup_path()
#------------------------------------------------------------------------------------------------------
import threading
import keyboard
from time import sleep
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
        eeConfig.load_config() 

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
                if keyboard.is_pressed('s'):
                    print("\nSaving Position:")
                    print("1: Scooping position")
                    print("2: Grab position")
                    print("3: Lob position")
               
                    while True:
                       if keyboard.is_pressed('1'):
                           position_type = 'scoop'
                           break
                       elif keyboard.is_pressed('2'):
                           position_type = 'grab'
                           break
                       elif keyboard.is_pressed('3'):
                           position_type = 'lob'
                           break
                       sleep(0.1)

                    current_position = Gripper.GetMotorPosition()
                    eeConfig.update_position(position_type, current_position)
                    print(f"\nNew {position_type} position Updated : {current_position}")

                    sleep(0.3)
           
                elif keyboard.is_pressed('q'):
                    print("\n Terminate Process")
                    self.running = False
                    break
                
            except Exception as e:
                print(f"Error while Processing Key Input: {e}")
                break

    def start(self):
        try:
            self.gripper_thread = threading.Thread(
                target=self.gripper_thread_function,
                name="GripperThread"
            )
            self.gripper_thread.daemon = True
            self.gripper_thread.start()

            keyboard_thread = threading.Thread(
                target=self.handle_keyboard_input,
                name="KeyboardThread"
            )
            keyboard_thread.daemon = True
            keyboard_thread.start()

            self.gripper_thread.join()
            keyboard_thread.join()

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