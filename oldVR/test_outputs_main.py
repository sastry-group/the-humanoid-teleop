#testing

from television import JointTracking
from VRDataCollection import ChangeOfBasis

import time

if __name__ == '__main__':
    COB = ChangeOfBasis()
    JT = JointTracking(cert_file=r"C:\Users\caitl\arm_position_tracking\cert.pem", key_file=r"C:\Users\caitl\arm_position_tracking\key.pem", use_ngrok=False)
    while True:
        #head_matrix, left_wrist_matrix, right_wrist_matrix, left_fingers_matrix, right_fingers_matrix = COB.get_data()
        head_matrix = JT.head()
        left_wrist_matrix = JT.left_hand()
        right_wrist_matrix = JT.right_hand()
        left_fingers_matrix = JT.left_landmarks()
        right_fingers_matrix = JT.right_landmarks()

        print("Head Matrix: ", head_matrix)
        print("Left Wrist: ", left_wrist_matrix)
        print("Right Wrist: ", right_wrist_matrix)
        print("Left Fingers: ", left_fingers_matrix)
        print("Right Fingers: ", right_fingers_matrix)

        time.sleep(0.01)
