from television import JointTracking
import numpy as np
from HelperCalculationFunctions import *

class ChangeOfBasis:
    """
    Class where we explicitly find matricies of desired joints and change 
    the basis to be in the correct frame.
    
    """
    def __init__(self):
        #self.tv = JointTracking(cert_file="../cert.pem", key_file="../key.pem", use_ngrok=False)
        self.tv = JointTracking(cert_file=r"C:\Users\caitl\arm_position_tracking\cert.pem", key_file=r"C:\Users\caitl\arm_position_tracking\key.pem", use_ngrok=False)

        #Default matricies 
        self.vuer_head_mat = np.array([[1, 0, 0, 0],
                                       [0, 1, 0, 1.5],
                                       [0, 0, 1, -0.2],
                                       [0, 0, 0, 1]])
        self.vuer_right_wrist_mat = np.array([[1, 0, 0, 0.5],
                                       [0, 1, 0, 1],
                                       [0, 0, 1, -0.5],
                                       [0, 0, 0, 1]])  
        self.vuer_left_wrist_mat = np.array([[1, 0, 0, -0.5],
                                             [0, 1, 0, 1],
                                             [0, 0, 1, -0.5],
                                             [0, 0, 0, 1]])
        
    def get_data(self):
        self.vuer_head_mat = mat_update(self.vuer_head_mat, self.tv.head.copy())
        self.vuer_left_wrist_mat = mat_update(self.vuer_left_wrist_mat, self.tv.left_hand.copy())
        self.vuer_right_wrist_mat = mat_update(self.vuer_right_wrist_mat, self.tv.right_hand.copy())

        #Change basis, from Y point up to Z pointed up. OpenXR gives Y up convention.
        head_mat = mat_y_to_z_up @ self.vuer_head_mat @ fast_mat_inv(mat_y_to_z_up)
        left_wrist_mat = mat_y_to_z_up @ self.vuer_left_wrist_mat @ fast_mat_inv(mat_y_to_z_up)
        right_wrist_mat = mat_y_to_z_up @ self.vuer_right_wrist_mat @ fast_mat_inv(mat_y_to_z_up)

        #Change basis to Inspire coordinates for hand manipulation
        rel_left_wrist_mat = fast_mat_inv(left_wrist_mat) @ hand_to_Inspire
        rel_left_wrist_mat[0:3, 3] = rel_left_wrist_mat[0:3, 3] - head_mat[0:3, 3] #
        rel_right_wrist_mat = fast_mat_inv(right_wrist_mat) @ hand_to_Inspire
        rel_right_wrist_mat[0:3, 3] = rel_right_wrist_mat[0:3, 3] - head_mat[0:3, 3]

        #Similarly, find finger positions within Inspire coordinate frame
        left_fingers_mat = np.concatenate([self.tv.left_landmarks_pos.copy().T, np.ones((1, self.tv.left_landmarks.shape[0]))])
        right_fingers_mat = np.concatenate([self.tv.right_landmarks_pos.copy().T, np.ones((1, self.tv.right_landmarks_pos.shape[0]))])

        rel_left_fingers = fast_mat_inv(left_wrist_mat) @ left_fingers_mat
        rel_left_fingers = (hand_to_Inspire.T @ rel_left_fingers)[0:3, :].T
        rel_right_fingers = fast_mat_inv(right_wrist_mat) @ right_fingers_mat
        rel_right_fingers = (hand_to_Inspire.T @ rel_right_fingers)[0:3, :].T

        return head_mat, rel_right_wrist_mat, rel_right_wrist_mat, rel_left_fingers, rel_right_fingers







