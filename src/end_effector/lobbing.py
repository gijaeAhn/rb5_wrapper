from config import eeConfig

import numpy as np
from time import sleep
from GRIPPER import Gripper
from GRIPPER.Gripper import FREQUENCY
from datetime import datetime


def Lobbing():

    print("    [INIT LOBBING]    ")
    
    def check_encoder_difference(current, previous):
        difference = abs(current - previous)
        diffSum = difference[0] + difference[1]
        print("Diff Sum : ", diffSum)
        return diffSum > eeConfig.ENCODER_THRESHOLD
    
    try:
        # Gripper.SetControlState()
        # config = eeConfig.LOBBING['scoop']
        # Gripper.SetMotorPosition(config.position)
        # print("SET MOTOR SCOOP")
        # sleep(eeConfig.TIMING['initial_delay'])
        # Gripper.SetStiffness(config.stiffness)
        # Gripper.SetVelocityGain(config.velocity_gain)


        
        # time_step = 0.0
        # encoder_values = np.zeros(2)
        # prev_encoder_values = np.zeros(2)
        # first_grab_phase = True
        
        while True :
            print("CURRENT GRIPPER POSITION : ", Gripper.GetMotorPosition())
            sleep(1)
        #     while time_step < eeConfig.TIMING['lobbing_duration']:
        #         while (first_grab_phase and 
        #                time_step < eeConfig.TIMING['max_grab_duration'] and 
        #                Gripper.GetMotorPosition()[1] < eeConfig.MOTOR_Y_THRESHOLD):

        #             encoder_values = Gripper.GetEncoderValue()

        #             if check_encoder_difference(encoder_values, prev_encoder_values):
        #                 grab_config = eeConfig.LOBBING['grab']
        #                 Gripper.SetStiffness(grab_config.stiffness)
        #                 Gripper.SetVelocityGain(grab_config.velocity_gain)
        #                 Gripper.SetMotorPosition(grab_config.position)
        #                 print("SET MOTOR GRAB")

        #             time_step += 1/FREQUENCY
        #             sleep(1/FREQUENCY)
        #             prev_encoder_values = encoder_values.copy()

        #         first_grab_phase = False
        #         lob_config = eeConfig.LOBBING['lob']
        #         Gripper.SetVelocityGain(lob_config.velocity_gain)
        #         Gripper.SetMotorPosition(lob_config.position)
        #         print("SET MOTOR LOB")

        #         time_step += 1/FREQUENCY
        #         sleep(1/FREQUENCY)
                
        #     # After One Cycle
        #     if time_step >= eeConfig.TIMING['lobbing_duration']:
        #         if eeConfig._doOnce:
        #             break 
        #         else:
        #             time_step = 0.0
        #             encoder_values = np.zeros(2)
        #             prev_encoder_values = np.zeros(2)
        #             first_grab_phase = True
        #             config = eeConfig.LOBBING['scoop']
        #             Gripper.SetMotorPosition(config.position)
        #             sleep(eeConfig.TIMING['initial_delay'])
        #             Gripper.SetStiffness(config.stiffness)
        #             Gripper.SetVelocityGain(config.velocity_gain)

    except Exception as e:
        print(f"Error during Lobbing: {str(e)}")
    finally:
        Gripper.SetIdleState()