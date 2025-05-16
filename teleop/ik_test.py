import numpy as np
import time
import argparse
# import cv2
from multiprocessing import shared_memory, Array, Lock, Process
import threading

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from teleop.preprocessing.processing import PreProcessing
from teleop.robot_control.robot_arm import H1_ArmController
from teleop.robot_control.robot_arm_ik import H1_ArmIK
from teleop.robot_control.arm_control import Custom
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from teleop.robot_control.robot_hand_inspire import Inspire_Controller

def format_dual_arm(joint_array):
    """
    joint_array: array‑like of shape (8,) or (1,8) or (1, 4)
    returns: string "lspitch {v0}; lsroll {v1}; …; relbow {v7}"
    """
    # ensure it’s a flat 1‑D sequence of length 8
    arr = np.asarray(joint_array).flatten()
    if arr.shape[0] != 8 and arr.shape[0] != 4:
        raise ValueError("Expected 8 or 4 joint values, got shape %s" % (arr.shape,))

    # names = [
    #     "lspitch", "lsroll", "lsyaw", "lelbow",
    #     "rspitch", "rsroll", "rsyaw", "relbow"
    # ]
    names = [
        "rspitch", "rsroll", "rsyaw", "relbow"
    ]

    # format each as "name value" with three decimal places, join with "; "
    parts = [f"{n} {arr[i]:.4f}" for i, n in enumerate(names)]
    return "; ".join(parts)

if __name__ == '__main__':
    input("Press Enter to start the program...")
    # television: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
    tv_wrapper = PreProcessing()
    print("Television Wrapper Initialized")
    # arm control
    # arm_ctrl = H1_ArmController()
    print("Arm Controller Initialized")
    # arm_ik = H1_ArmIK()
    print("Arm IK Initialized")
    # Initialize the channel
    # custom = Custom()
    # print("Custom Object Initialized")
    # custom.Init()
    # print("Custom Initialized")
    # custom.Start()
    # print("Custom Started")

    left_hand_array = Array('d', 75, lock = True)          # [input]
    right_hand_array = Array('d', 75, lock = True)         # [input]
    dual_hand_data_lock = Lock()
    dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
    dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
    hand_ctrl = Inspire_Controller(left_hand_array, right_hand_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array)
    
    try:
        # user_input = input("Please enter the start signal (enter 'r' to start the subsequent program):\n")
        # if user_input.lower() == 'r':
        #     arm_ctrl.speed_gradual_max()

        running = True
        while running:
            start_time = time.time()
            head_rmat, left_wrist, right_wrist, left_hand, right_hand = tv_wrapper.process()

            # send hand skeleton data to hand_ctrl.control_process
            left_hand_array[:] = left_hand.flatten()
            right_hand_array[:] = right_hand.flatten()

            if np.linalg.norm(left_hand_array) > 0.1 and np.linalg.norm(right_hand_array) > 0.1:

                # #get current state data.
                # # current_lr_arm_q  = np.zeros(8, dtype=float) 
                # # current_lr_arm_dq = np.zeros(8, dtype=float) 
                # current_lr_arm_q = arm_ctrl.get_current_dual_arm_q()
                # current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()
                # print("Current Arm Q: ", current_lr_arm_q)
                # print("Current Arm DQ: ", current_lr_arm_dq)    

                # # solve ik using motor data and wrist pose, then use ik results to control arms.
                # # time_ik_start = time.time()
                # sol_q, sol_tauff  = arm_ik.solve_ik(left_wrist, right_wrist, current_lr_arm_q, current_lr_arm_dq)
                # # sol_q, sol_tauff  = arm_ik.solve_ik(left_wrist, right_wrist, None, None)
                # # time_ik_end = time.time()
                # print("Solved State: ", sol_q)
                # print("Solved Torque: ", sol_tauff)
        
                time.sleep(0.1)
                # cmd_str = format_dual_arm(sol_q[4:])
                # for _ in range(10):
                #     custom.process_command(cmd_str)
                # arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)


    except KeyboardInterrupt:
        print("KeyboardInterrupt, exiting program...")
    # finally:
        # arm_ctrl.ctrl_dual_arm_go_home()
