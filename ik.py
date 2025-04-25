import numpy as np
from scipy.spatial.transform import Rotation as R

class HumanoidIK:
    def __init__(self):
        # Robot parameters
        self.shoulder_to_elbow_length = 0.30  # Example value in meters
        self.elbow_to_wrist_length = 0.30     # Example value in meters
        
    def compute_arm_ik(self, wrist_position, is_right_arm=True):
        """
        Compute inverse kinematics for one arm
        Args:
            wrist_position: np.array([x, y, z]) - target wrist position in robot base frame
            is_right_arm: bool - True for right arm, False for left arm
        Returns:
            joint_angles: dict containing shoulder and elbow angles in radians
        """
        # Shoulder position (assuming origin at shoulder)
        shoulder_pos = np.zeros(3)
        
        # Vector from shoulder to target wrist position
        target_vector = wrist_position - shoulder_pos
        
        # Total reach distance
        reach_distance = np.linalg.norm(target_vector)
        
        # Check if target is reachable
        max_reach = self.shoulder_to_elbow_length + self.elbow_to_wrist_length
        if reach_distance > max_reach:
            # Target is too far, extend arm in target direction
            target_vector = target_vector * (max_reach / reach_distance)
            reach_distance = max_reach
            
        # Compute elbow angle using law of cosines
        cos_elbow = (reach_distance**2 - self.shoulder_to_elbow_length**2 - 
                    self.elbow_to_wrist_length**2) / (
                    -2 * self.shoulder_to_elbow_length * self.elbow_to_wrist_length)
        elbow_angle = np.arccos(np.clip(cos_elbow, -1.0, 1.0))
        
        # Compute shoulder angles
        shoulder_pitch = np.arctan2(target_vector[2], 
                                  np.sqrt(target_vector[0]**2 + target_vector[1]**2))
        shoulder_yaw = np.arctan2(target_vector[1], target_vector[0])
        
        return {
            'shoulder_yaw': shoulder_yaw,
            'shoulder_pitch': shoulder_pitch,
            'elbow': elbow_angle
        }
    
    def process_vr_data(self, head_pos, left_wrist_pos, right_wrist_pos, left_fingers, right_fingers):
        """
        Process VR tracking data and compute robot joint angles
        Args:
            head_pos: np.array([x, y, z]) - head position
            left_wrist_pos: np.array([x, y, z]) - left wrist position
            right_wrist_pos: np.array([x, y, z]) - right wrist position
            left_fingers: finger state for left hand
            right_fingers: finger state for right hand
        Returns:
            dict containing joint angles for both arms
        """
        # Transform VR coordinates to robot frame if needed
        # This is a placeholder - implement actual transformation based on your setup
        
        left_arm_angles = self.compute_arm_ik(left_wrist_pos, is_right_arm=False)
        right_arm_angles = self.compute_arm_ik(right_wrist_pos, is_right_arm=True)
        
        return {
            'left_arm': left_arm_angles,
            'right_arm': right_arm_angles,
            'left_gripper': left_fingers,
            'right_gripper': right_fingers
        }

def main():
    ik_solver = HumanoidIK()
    
    # Example usage (replace with actual VR data from television.py)
    head_pos = np.array([0, 0, 0])
    left_wrist = np.array([-0.3, 0.2, 0.1])
    right_wrist = np.array([0.3, 0.2, 0.1])
    left_fingers = 0.5
    right_fingers = 0.5
    
    joint_angles = ik_solver.process_vr_data(head_pos, left_wrist, right_wrist, 
                                           left_fingers, right_fingers)
    print("Computed joint angles:", joint_angles)

if __name__ == "__main__":
    main()