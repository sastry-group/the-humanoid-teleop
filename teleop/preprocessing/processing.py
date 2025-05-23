import numpy as np
from TeleVision import TeleVision
from .constants import *
from utils.MatrixTools import mat_update, fast_mat_inv
from multiprocessing import shared_memory, Queue, Event

class PreProcessing:
    def __init__(self):
        resolution = (720, 1280)
        crop_size_w = 340  # (resolution[1] - resolution[0]) // 2
        crop_size_h = 270
        resolution_cropped = (resolution[0] - crop_size_h, resolution[1] - 2 * crop_size_w)  # 450 * 600
        img_shape = (2 * resolution_cropped[0], resolution_cropped[1], 3)  # 900 * 600
        shm = shared_memory.SharedMemory(create=True, size=np.prod(img_shape) * np.uint8().itemsize)

        # television
        cert_file = "cert.pem"
        key_file = "key.pem"
        image_queue = Queue()
        toggle_streaming = Event()
        self.tv = TeleVision()
        # self.tv = TeleVision(resolution_cropped, image_queue, toggle_streaming, cert_file=cert_file, key_file=key_file, stream_mode="webrtc")
        
    def process(self):
        """
        Pulled from tv_wrapper.py of Unitree's avp_teleoperate, Vuer's default coordinate system is same as OpenXR convention
        """

        # --------------------------------wrist-------------------------------------

        # TeleVision obtains a basis coordinate that is OpenXR Convention
        head_vuer_mat, head_flag = mat_update(const_head_vuer_mat, self.tv.head_matrix.copy())
        left_wrist_vuer_mat, left_wrist_flag  = mat_update(const_left_wrist_vuer_mat, self.tv.left_wrist.copy())
        right_wrist_vuer_mat, right_wrist_flag = mat_update(const_right_wrist_vuer_mat, self.tv.right_wrist.copy())

        # print("left_wrist_vuer_mat: ", self.tv.left_wrist.copy())
        # print("right_wrist_vuer_mat: ", self.tv.right_wrist.copy())

        # Change basis convention: VuerMat ((basis) OpenXR Convention) to WristMat ((basis) Robot Convention)
        # p.s. WristMat = T_{robot}_{openxr} * VuerMat * T_{robot}_{openxr}^T
        # Reason for right multiply fast_mat_inv(T_robot_openxr):
        #   This is similarity transformation: B = PAP^{-1}, that is B ~ A
        #   For example:
        #   - For a pose data T_r under the Robot Convention, left-multiplying WristMat means:
        #   - WristMat * T_r  ==>  T_{robot}_{openxr} * VuerMat * T_{openxr}_{robot} * T_r
        #   - First, transform to the OpenXR Convention (The function of T_{openxr}_{robot})
        #   - then, apply the rotation VuerMat in the OpenXR Convention (The function of VuerMat)
        #   - finally, transform back to the Robot Convention (The function of T_{robot}_{openxr})
        #   This results in the same rotation effect under the Robot Convention as in the OpenXR Convention.
        head_mat = T_robot_openxr @ head_vuer_mat @ fast_mat_inv(T_robot_openxr)
        left_wrist_mat  = T_robot_openxr @ left_wrist_vuer_mat @ fast_mat_inv(T_robot_openxr)
        right_wrist_mat = T_robot_openxr @ right_wrist_vuer_mat @ fast_mat_inv(T_robot_openxr)

        # Change wrist convention: WristMat ((Left Wrist) XR/AppleVisionPro Convention) to UnitreeWristMat((Left Wrist URDF) Unitree Convention)
        # Reason for right multiply (T_to_unitree_left_wrist) : Rotate 90 degrees counterclockwise about its own x-axis.
        # Reason for right multiply (T_to_unitree_right_wrist): Rotate 90 degrees clockwise about its own x-axis.
        unitree_left_wrist = left_wrist_mat @ (T_to_unitree_left_wrist if left_wrist_flag else np.eye(4))
        unitree_right_wrist = right_wrist_mat @ (T_to_unitree_right_wrist if right_wrist_flag else np.eye(4))

        # Transfer from WORLD to HEAD coordinate (translation only).
        unitree_left_wrist[0:3, 3]  = unitree_left_wrist[0:3, 3] - head_mat[0:3, 3]
        unitree_right_wrist[0:3, 3] = unitree_right_wrist[0:3, 3] - head_mat[0:3, 3]

        # --------------------------------hand------------ch-------------------------

        # Homogeneous, [xyz] to [xyz1]
        # p.s. np.concatenate([25,3]^T,(1,25)) ==> hand_vuer_mat.shape is (4,25)
        # Now under (basis) OpenXR Convention, mat shape like this:
        #    x0 x1 x2 ··· x23 x24
        #    y0 y1 y1 ··· y23 y24
        #    z0 z1 z2 ··· z23 z24
        #     1  1  1 ···   1   1
        left_hand_vuer_mat  = np.concatenate([self.tv.left_landmarks.copy().T, np.ones((1, self.tv.left_landmarks.shape[0]))])
        right_hand_vuer_mat = np.concatenate([self.tv.right_landmarks.copy().T, np.ones((1, self.tv.right_landmarks.shape[0]))])

        # Change basis convention: from (basis) OpenXR Convention to (basis) Robot Convention
        # Just a change of basis for 3D points. No rotation, only translation. No need to right-multiply fast_mat_inv(T_robot_openxr).
        left_hand_mat  = T_robot_openxr @ left_hand_vuer_mat
        right_hand_mat = T_robot_openxr @ right_hand_vuer_mat

        # Transfer from WORLD to WRIST coordinate. (this process under (basis) Robot Convention)
        # p.s.  HandMat_WristBased = WristMat_{wrold}_{wrist}^T * HandMat_{wrold}
        #       HandMat_WristBased = WristMat_{wrist}_{wrold}   * HandMat_{wrold}, that is HandMat_{wrist}
        left_hand_mat_wb  = fast_mat_inv(left_wrist_mat) @ left_hand_mat
        right_hand_mat_wb = fast_mat_inv(right_wrist_mat) @ right_hand_mat
        # Change hand convention: HandMat ((Left Hand) XR/AppleVisionPro Convention) to UnitreeHandMat((Left Hand URDF) Unitree Convention)
        # Reason for left multiply : T_to_unitree_hand @ left_hand_mat_wb ==> (4,4) @ (4,25) ==> (4,25), (4,25)[0:3, :] ==> (3,25), (3,25).T ==> (25,3)           
        # Now under (Left Hand URDF) Unitree Convention, mat shape like this:
        #    [x0, y0, z0]
        #    [x1, y1, z1]
        #    ···
        #    [x23,y23,z23] 
        #    [x24,y24,z24]               
        unitree_left_hand  = (T_to_unitree_hand @ left_hand_mat_wb)[0:3, :].T
        unitree_right_hand = (T_to_unitree_hand @ right_hand_mat_wb)[0:3, :].T

        # --------------------------------offset-------------------------------------

        head_rmat = head_mat[:3, :3]
        # The origin of the coordinate for IK Solve is the WAIST joint motor. You can use teleop/robot_control/robot_arm_ik.py Unit_Test to check it.
        # The origin of the coordinate of unitree_left_wrist is HEAD. So it is necessary to translate the origin of unitree_left_wrist from HEAD to WAIST.
        unitree_left_wrist[0, 3] +=0.15
        unitree_right_wrist[0,3] +=0.15
        unitree_left_wrist[2, 3] +=0.45
        unitree_right_wrist[2,3] +=0.45

        # print("unitree_left_wrist: ", unitree_left_wrist)
        # print("unitree_right_wrist: ", unitree_right_wrist)

        return head_rmat, unitree_left_wrist, unitree_right_wrist, unitree_left_hand, unitree_right_hand
    
if __name__ == "__main__":

    tv_wrapper = PreProcessing()
    while True:
        tv_wrapper.process()

        # print("Left Wrist:\n", tv_wrapper.process()[0])
        # print("Right Wrist:\n", tv_wrapper.process()[1])
        # print("Left Hand:\n", tv_wrapper.process()[2])
        # print("Right Hand:\n", tv_wrapper.process()[3])