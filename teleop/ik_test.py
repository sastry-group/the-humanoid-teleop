import numpy as np
import time
import argparse
import cv2
from multiprocessing import shared_memory, Array, Lock, Process
import threading

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from teleop.preprocessing.processing import PreProcessing
# from teleop.robot_control.robot_arm import H1_ArmController
from teleop.robot_control.robot_arm_ik import H1_ArmIK
# from teleop.robot_control.robot_hand_inspire import Inspire_Controller


if __name__ == '__main__':
    # television: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
    tv_wrapper = PreProcessing()

    # arm
    # arm_ctrl = H1_ArmController()
    arm_ik = H1_ArmIK()

    left_hand_array = Array('d', 75, lock = True)          # [input]
    right_hand_array = Array('d', 75, lock = True)         # [input]
    dual_hand_data_lock = Lock()
    dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
    dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
    # hand_ctrl = Inspire_Controller(left_hand_array, right_hand_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array)
    
    try:
        # user_input = input("Please enter the start signal (enter 'r' to start the subsequent program):\n")
        # if user_input.lower() == 'r':
            # arm_ctrl.speed_gradual_max()

        running = True
        while running:
            start_time = time.time()
            head_rmat, left_wrist, right_wrist, left_hand, right_hand = tv_wrapper.process()

            # send hand skeleton data to hand_ctrl.control_process
            left_hand_array[:] = left_hand.flatten()
            right_hand_array[:] = right_hand.flatten()
            # print("Left Hand:\n", len(left_hand_array)
            # print("Right Hand:\n", right_hand_array.shape)

            # get current state data.
            current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            # solve ik using motor data and wrist pose, then use ik results to control arms.
            time_ik_start = time.time()
            sol_q, sol_tauff  = arm_ik.solve_ik(left_wrist, right_wrist, current_lr_arm_q, current_lr_arm_dq)
            # sol_q, sol_tauff  = arm_ik.solve_ik(left_wrist, right_wrist, None, None)
            time_ik_end = time.time()
            print("Solved State: ", sol_q)
            # print("Solved Torque: ", sol_tauff)
            # arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

    except KeyboardInterrupt:
        print("KeyboardInterrupt, exiting program...")
    # finally:
        # arm_ctrl.ctrl_dual_arm_go_home()
