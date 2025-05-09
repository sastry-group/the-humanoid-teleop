import time
import sys
import os
import datetime
import colorama
from colorama import Fore, Style
import readline
import threading
import termios
import fcntl
import tty
import select
import glob

colorama.init(autoreset=True)

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient


H1_NUM_MOTOR = 20
PosStopF = 2.146e9
VelStopF = 16000.0

class H1JointIndex:
    # Right leg
    kRightHipYaw = 8
    kRightHipRoll = 0
    kRightHipPitch = 1
    kRightKnee = 2
    kRightAnkle = 11
    # Left leg
    kLeftHipYaw = 7
    kLeftHipRoll = 3
    kLeftHipPitch = 4
    kLeftKnee = 5
    kLeftAnkle = 10

    kWaistYaw = 6

    kNotUsedJoint = 9

    # Right arm
    kRightShoulderPitch = 12
    kRightShoulderRoll = 13
    kRightShoulderYaw = 14
    kRightElbow = 15
    # Left arm
    kLeftShoulderPitch = 16
    kLeftShoulderRoll = 17
    kLeftShoulderYaw = 18
    kLeftElbow = 19

# Joint name to index mapping for user-friendly commands
JOINT_NAME_MAP = {
    # Abbreviations to H1JointIndex attributes
    "rhyaw": "kRightHipYaw",
    "rhroll": "kRightHipRoll",
    "rhpitch": "kRightHipPitch",
    "rknee": "kRightKnee",
    "rankle": "kRightAnkle",
    
    "lhyaw": "kLeftHipYaw",
    "lhroll": "kLeftHipRoll",
    "lhpitch": "kLeftHipPitch",
    "lknee": "kLeftKnee",
    "lankle": "kLeftAnkle",
    
    "waist": "kWaistYaw",
    
    "rspitch": "kRightShoulderPitch",
    "rsroll": "kRightShoulderRoll",
    "rsyaw": "kRightShoulderYaw",
    "relbow": "kRightElbow",
    
    "lspitch": "kLeftShoulderPitch",
    "lsroll": "kLeftShoulderRoll",
    "lsyaw": "kLeftShoulderYaw",
    "lelbow": "kLeftElbow",
    
    # Full names for better readability
    "right_hip_yaw": "kRightHipYaw",
    "right_hip_roll": "kRightHipRoll",
    "right_hip_pitch": "kRightHipPitch",
    "right_knee": "kRightKnee",
    "right_ankle": "kRightAnkle",
    
    "left_hip_yaw": "kLeftHipYaw",
    "left_hip_roll": "kLeftHipRoll",
    "left_hip_pitch": "kLeftHipPitch",
    "left_knee": "kLeftKnee",
    "left_ankle": "kLeftAnkle",
    
    "torso": "kWaistYaw",
    
    "right_shoulder_pitch": "kRightShoulderPitch",
    "right_shoulder_roll": "kRightShoulderRoll",
    "right_shoulder_yaw": "kRightShoulderYaw",
    "right_elbow": "kRightElbow",
    
    "left_shoulder_pitch": "kLeftShoulderPitch",
    "left_shoulder_roll": "kLeftShoulderRoll",
    "left_shoulder_yaw": "kLeftShoulderYaw",
    "left_elbow": "kLeftElbow",
}

# Joint position limits (in radians) [min, max]
JOINT_POSITION_LIMITS = {
    H1JointIndex.kRightHipYaw: [-0.43, 0.43],
    H1JointIndex.kRightHipRoll: [-0.43, 0.43],
    H1JointIndex.kRightHipPitch: [-3.14, 2.53],
    H1JointIndex.kRightKnee: [-0.26, 2.05],
    H1JointIndex.kRightAnkle: [-0.87, 0.52],
    H1JointIndex.kLeftHipYaw: [-0.43, 0.43],
    H1JointIndex.kLeftHipRoll: [-0.43, 0.43],
    H1JointIndex.kLeftHipPitch: [-3.14, 2.53],
    H1JointIndex.kLeftKnee: [-0.26, 2.05],
    H1JointIndex.kLeftAnkle: [-0.87, 0.52],
    H1JointIndex.kWaistYaw: [-2.35, 2.35],
    H1JointIndex.kRightShoulderPitch: [-2.87, 2.87],
    H1JointIndex.kRightShoulderRoll: [-3.11, 0.34],
    H1JointIndex.kRightShoulderYaw: [-4.45, 1.3],
    H1JointIndex.kRightElbow: [-1.25, 2.61],
    H1JointIndex.kLeftShoulderPitch: [-2.87, 2.87],
    H1JointIndex.kLeftShoulderRoll: [-0.34, 3.11],
    H1JointIndex.kLeftShoulderYaw: [-1.3, 4.45],
    H1JointIndex.kLeftElbow: [-1.25, 2.61],
}

class Custom:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.01  
        self.duration_ = 60.0    
        self.counter_ = 0
        self.kp_low_ = 30.0
        self.kp_high_ = 100.0
        self.kd_low_ = 1.5
        self.kd_high_ = 5.0
        self.low_cmd = unitree_go_msg_dds__LowCmd_()  
        self.InitLowCmd()
        self.low_state = None 
        self.crc = CRC()

    def Init(self):
        # ChannelFactoryInitialize(0)
        # # create publisher #
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        # # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

        # initialize subscribe thread
        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)
            
        # Remove the recurring RPY reporting to avoid terminal flooding
        # We'll only call ReportRPY manually after command execution
        # self.report_rpy_ptr_ = RecurrentThread(
        #     interval=0.1, target=self.ReportRPY, name="report_rpy"
        # )
        # self.report_rpy_ptr_.Start()

    def is_weak_motor(self,motor_index):
        return motor_index in {
            H1JointIndex.kLeftAnkle,
            H1JointIndex.kRightAnkle,
            H1JointIndex.kRightShoulderPitch,
            H1JointIndex.kRightShoulderRoll,
            H1JointIndex.kRightShoulderYaw,
            H1JointIndex.kRightElbow,
            H1JointIndex.kLeftShoulderPitch,
            H1JointIndex.kLeftShoulderRoll,
            H1JointIndex.kLeftShoulderYaw,
            H1JointIndex.kLeftElbow,
        }

    def InitLowCmd(self):
        self.low_cmd.head[0] = 0xFE
        self.low_cmd.head[1] = 0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(H1_NUM_MOTOR):
            if self.is_weak_motor(i):
                self.low_cmd.motor_cmd[i].mode = 0x01 
            else:
                self.low_cmd.motor_cmd[i].mode = 0x0A 
            self.low_cmd.motor_cmd[i].q= PosStopF
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = VelStopF
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

    def ReportRPY(self):
        """Report Roll, Pitch, Yaw and joint positions in a formatted way"""
        # Suppress unnecessary flooding by removing print statements
        # Print formatted RPY and joint positions only after command execution
        rpy = [round(val, 4) for val in self.low_state.imu_state.rpy]
        print(f"{Fore.CYAN}RPY: {rpy}{Style.RESET_ALL}")

        left_elbow_q = round(self.low_state.motor_state[H1JointIndex.kLeftElbow].q, 4)
        right_elbow_q = round(self.low_state.motor_state[H1JointIndex.kRightElbow].q, 4)
        left_shoulder_pitch_q = round(self.low_state.motor_state[H1JointIndex.kLeftShoulderPitch].q, 4)
        right_shoulder_pitch_q = round(self.low_state.motor_state[H1JointIndex.kRightShoulderPitch].q, 4)
        
        print(f"{Fore.CYAN}Joint Positions:{Style.RESET_ALL}")
        print(f"  {Fore.YELLOW}Left Elbow: {left_elbow_q}{Style.RESET_ALL}")
        print(f"  {Fore.YELLOW}Right Elbow: {right_elbow_q}{Style.RESET_ALL}")
        print(f"  {Fore.YELLOW}Left Shoulder Pitch: {left_shoulder_pitch_q}{Style.RESET_ALL}")
        print(f"  {Fore.YELLOW}Right Shoulder Pitch: {right_shoulder_pitch_q}{Style.RESET_ALL}")

    def reset_all_joints(self):
        """Reset all joints to position 0 gradually"""
        print(f"{Fore.CYAN}Resetting all joints to position 0...{Style.RESET_ALL}")
        
        if self.low_state is None:
            print("No joint state information available yet")
            return False

        # Get current positions
        current_positions = [self.low_state.motor_state[i].q for i in range(H1_NUM_MOTOR)]
        
        # More steps and longer delay for slower movement
        num_steps = 100
        print(f"{Fore.CYAN}Resetting joints to 0...{Style.RESET_ALL}")
        
        for step in range(num_steps + 1):
            # Calculate progress percentage
            progress = step / num_steps
            
            # Update each joint
            for i in range(H1_NUM_MOTOR):
                # Skip unused and ankle joints
                if i == H1JointIndex.kNotUsedJoint or i in [H1JointIndex.kLeftAnkle, H1JointIndex.kRightAnkle]:
                    continue
                
                # Linear interpolation to 0
                new_pos = current_positions[i] * (1 - progress)
                
                # Update command with reduced gains for smoother movement
                self.low_cmd.motor_cmd[i].q = new_pos
                self.low_cmd.motor_cmd[i].dq = 0.0
                self.low_cmd.motor_cmd[i].tau = 0.0
                self.low_cmd.motor_cmd[i].kp = self.kp_low_ * 0.5  # Reduced gains
                self.low_cmd.motor_cmd[i].kd = self.kd_low_ * 0.5  # Reduced gains
            
            # Show progress every 10%
            if step % 10 == 0:
                print(f"\r{Fore.YELLOW}Progress: {progress*100:3.0f}%{Style.RESET_ALL}", end='')
            
            # Longer delay between steps
            time.sleep(0.05)
        
        print(f"\n{Fore.GREEN}Reset complete{Style.RESET_ALL}")
        

        return True
        
    def direct_joint_control(self, joint_name):
        """Control joint position directly with arrow keys"""
        if joint_name not in JOINT_NAME_MAP:
            print(f"Error: Unknown joint '{joint_name}'")
            print("Available joints: " + ", ".join(sorted(set(JOINT_NAME_MAP.keys()))))
            return False
            
        # Get joint index
        joint_attr = JOINT_NAME_MAP[joint_name]
        joint_idx = getattr(H1JointIndex, joint_attr)
                
        # Get current position
        if self.low_state is None:
            current_pos = 0.0
        else:
            current_pos = self.low_state.motor_state[joint_idx].q
            
        # Set step size based on joint type
        if joint_idx in [H1JointIndex.kLeftElbow, H1JointIndex.kRightElbow]:
            step_size = 0.05  # Smaller steps for elbow
        else:
            step_size = 0.02  # Default step size
            
        min_pos, max_pos = JOINT_POSITION_LIMITS[joint_idx]
        print(f"{Fore.CYAN}Direct control mode for {joint_name}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}Use left/right arrow keys to adjust position. Press 'x' to exit.{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}Current position: {current_pos:.4f} (Limits: {min_pos:.4f} to {max_pos:.4f}){Style.RESET_ALL}")
        
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # Set terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            
            # Function to make stdin non-blocking
            fd = sys.stdin.fileno()
            fl = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
            
            # Control loop
            while True:
                # Check if there's input available
                r, w, e = select.select([sys.stdin], [], [], 0.1)
                
                if r:
                    key = sys.stdin.read(1)
                    
                    # Exit on 'x'
                    if key == 'x':
                        print(f"\n{Fore.GREEN}Exiting direct control mode{Style.RESET_ALL}")
                        break
                        
                    # Handle arrow keys
                    if key == '\x1b':  # Escape sequence
                        sys.stdin.read(1)  # Skip the '[' character
                        arrow = sys.stdin.read(1)
                        
                        if arrow == 'C':  # Right arrow
                            new_pos = current_pos + step_size
                            # Check joint limits
                            min_pos, max_pos = JOINT_POSITION_LIMITS[joint_idx]
                            if new_pos > max_pos:
                                print(f"\r{Fore.RED}Cannot move further: At maximum limit {max_pos:.4f}{Style.RESET_ALL}", end='')
                                sys.stdout.flush()
                                continue
                            current_pos = new_pos
                            print(f"\r{Fore.YELLOW}Position: {current_pos:.4f} (↑){Style.RESET_ALL}", end='')
                            # Apply new position
                            self.low_cmd.motor_cmd[joint_idx].q = current_pos
                            self.low_cmd.motor_cmd[joint_idx].dq = 0.0
                            self.low_cmd.motor_cmd[joint_idx].tau = 0.0
                            self.low_cmd.motor_cmd[joint_idx].kp = self.kp_low_
                            self.low_cmd.motor_cmd[joint_idx].kd = self.kd_low_
                            sys.stdout.flush()
                            
                        elif arrow == 'D':  # Left arrow
                            new_pos = current_pos - step_size
                            # Check joint limits
                            min_pos, max_pos = JOINT_POSITION_LIMITS[joint_idx]
                            if new_pos < min_pos:
                                print(f"\r{Fore.RED}Cannot move further: At minimum limit {min_pos:.4f}{Style.RESET_ALL}", end='')
                                sys.stdout.flush()
                                continue
                            current_pos = new_pos
                            print(f"\r{Fore.YELLOW}Position: {current_pos:.4f} (↓){Style.RESET_ALL}", end='')
                            # Apply new position
                            self.low_cmd.motor_cmd[joint_idx].q = current_pos
                            self.low_cmd.motor_cmd[joint_idx].dq = 0.0
                            self.low_cmd.motor_cmd[joint_idx].tau = 0.0
                            self.low_cmd.motor_cmd[joint_idx].kp = self.kp_low_
                            self.low_cmd.motor_cmd[joint_idx].kd = self.kd_low_
                            sys.stdout.flush()
                
                # Sleep briefly to avoid consuming too much CPU
                time.sleep(0.05)
                
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("\n")  # Add newline after exiting
            return True
   
    def process_command(self, cmd_str):
        """Process a command string, allowing for shortcuts and default values."""
        try:
            # Split command into parts
            parts = cmd_str.strip().split()
            if not parts:
                return False

            # Check for batch commands
            if ';' in cmd_str:
                commands = cmd_str.split(';')
                for command in commands:
                    self.process_command(command.strip())
                return True

            # Repeat last command shortcut
            if cmd_str == '!!':
                if hasattr(self, 'last_command'):
                    print(f"Repeating last command: {self.last_command}")
                    return self.process_command(self.last_command)
                else:
                    print("No last command to repeat.")
                    return False

            # Help shortcut
            if cmd_str == '?':
                print_help()
                return True
                
            # Reset command
            if cmd_str == 'reset':
                return self.reset_all_joints()
                              
            # Show all joint positions command
            if cmd_str == 'joints':
                if self.low_state is None:
                    print("No joint state information available yet")
                    return True
                print(f"{Fore.CYAN}Current Joint Positions:{Style.RESET_ALL}")
                for joint_name, joint_attr in sorted(JOINT_NAME_MAP.items()):
                    if joint_name.startswith('_'): continue  # Skip special entries
                    joint_idx = getattr(H1JointIndex, joint_attr)
                    pos = self.low_state.motor_state[joint_idx].q
                    min_pos, max_pos = JOINT_POSITION_LIMITS[joint_idx]
                    # Only show the first occurrence of each joint (skip aliases)
                    if joint_name == next(k for k,v in JOINT_NAME_MAP.items() if v == joint_attr):
                        print(f"{Fore.GREEN}{joint_name:20}{Style.RESET_ALL}: {pos:7.4f} (Limits: {min_pos:7.4f} to {max_pos:7.4f})")
                return True
                

            # Store last command
            self.last_command = cmd_str

            # Check if it's just a joint name for direct control
            if len(parts) == 1 and parts[0] in JOINT_NAME_MAP:
                return self.direct_joint_control(parts[0])

            # Default parameter assumption
            if len(parts) == 2:
                joint_name, value = parts
                param = 'q'  # Default to position
                value = float(value)
            elif len(parts) == 3:
                joint_name, param, value = parts
                value = float(value)
            else:
                print("Error: Command must be in format: <joint> <param> <value> or <joint> <value>")
                return False

            # Look up joint name in the mapping
            if joint_name not in JOINT_NAME_MAP:
                print(f"Error: Unknown joint '{joint_name}'")
                print("Available joints: " + ", ".join(sorted(set(JOINT_NAME_MAP.keys()))))
                return False

            # Get the H1JointIndex attribute name
            joint_attr = JOINT_NAME_MAP[joint_name]
            joint_idx = getattr(H1JointIndex, joint_attr)

            # Update the appropriate parameter
            if param in ['q', 'dq', 'tau', 'kp', 'kd']:
                # Check for joint position limits
                if param == 'q':
                    min_pos, max_pos = JOINT_POSITION_LIMITS[joint_idx]
                    if value < min_pos:
                        print(f"Warning: Position {value} is below the minimum limit {min_pos} for joint {joint_name}.")
                        value = min_pos
                    elif value > max_pos:
                        print(f"Warning: Position {value} is above the maximum limit {max_pos} for joint {joint_name}.")
                        value = max_pos
                setattr(self.low_cmd.motor_cmd[joint_idx], param, value)
                print(f"Set {joint_name}.{param} = {value}")

                # Set default values for other parameters if setting position
                if param == 'q':
                    self.low_cmd.motor_cmd[joint_idx].dq = 0.0  # Default speed
                    self.low_cmd.motor_cmd[joint_idx].tau = 0.0  # Default torque
                    self.low_cmd.motor_cmd[joint_idx].kp = self.kp_low_  # Default position gain
                    self.low_cmd.motor_cmd[joint_idx].kd = self.kd_low_  # Default velocity gain

                # Print current joint state if available
                if self.low_state is not None:
                    self.print_joint_state(joint_idx)
                
                # Report RPY and joint positions
                self.ReportRPY()
                return True
            else:
                print(f"Error: Unknown parameter '{param}'. Use: q, dq, tau, kp, or kd")
                return False

        except Exception as e:
            print(f"Error processing command: {str(e)}")
            return False

    def print_joint_state(self, joint_idx):
        """Print detailed state information for a specific joint"""
        if self.low_state is None:
            print("No state data available yet")
            return
            
        # Find the joint name for display
        joint_attr = None
        for attr in dir(H1JointIndex):
            if attr.startswith('k') and getattr(H1JointIndex, attr) == joint_idx:
                joint_attr = attr
                break
                
        if joint_attr is None:
            print(f"Joint index {joint_idx} not found in H1JointIndex")
            return
            
        # Print joint state information with color
        print(f"\n{Fore.CYAN}Current state for {joint_attr}:{Style.RESET_ALL}")
        motor_state = self.low_state.motor_state[joint_idx]
        print(f"  {Fore.YELLOW}Position (q):       {motor_state.q:.4f}{Style.RESET_ALL}")
        print(f"  {Fore.YELLOW}Velocity (dq):      {motor_state.dq:.4f}{Style.RESET_ALL}")
        # Only display torque if available
        if hasattr(motor_state, 'tau'):
            print(f"  {Fore.YELLOW}Torque (tau):       {motor_state.tau:.4f}{Style.RESET_ALL}")
        print(f"  {Fore.YELLOW}Temperature:        {motor_state.temperature:.1f}°C{Style.RESET_ALL}")
        print(f"  {Fore.YELLOW}Mode:               {motor_state.mode}{Style.RESET_ALL}")
        
        # Print command information
        print(f"\n{Fore.CYAN}Current command for {joint_attr}:{Style.RESET_ALL}")
        motor_cmd = self.low_cmd.motor_cmd[joint_idx]
        print(f"  {Fore.GREEN}Position (q):       {motor_cmd.q:.4f}{Style.RESET_ALL}")
        print(f"  {Fore.GREEN}Velocity (dq):      {motor_cmd.dq:.4f}{Style.RESET_ALL}")
        print(f"  {Fore.GREEN}Torque (tau):       {motor_cmd.tau:.4f}{Style.RESET_ALL}")
        print(f"  {Fore.GREEN}Position gain (kp): {motor_cmd.kp:.4f}{Style.RESET_ALL}")
        print(f"  {Fore.GREEN}Velocity gain (kd): {motor_cmd.kd:.4f}{Style.RESET_ALL}")
        print(f"  {Fore.GREEN}Mode:               {motor_cmd.mode}{Style.RESET_ALL}")

    def LowCmdWrite(self):
        """Write the current low-level command to the robot"""
        # Update CRC and publish command
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)

def print_help():
    """Print help information about available commands"""
    print(f"\n{Fore.CYAN}=== H1 Robot Remote Control ==={Style.RESET_ALL}")
    print(f"\n{Fore.YELLOW}Joint Control Commands:{Style.RESET_ALL}")
    print("  <joint_name> <position>     - Set joint position (radians)")
    print("  <joint_name>               - Enter direct control mode for a joint (use arrow keys)")
    
    print(f"\n{Fore.YELLOW}Available Joints:{Style.RESET_ALL}")
    print("  Legs (short): rhyaw, rhroll, rhpitch, rknee, rankle, lhyaw, lhroll, lhpitch, lknee, lankle")
    print("  Legs (full): right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee, right_ankle,")
    print("               left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee, left_ankle")
    print("  Arms (short): rspitch, rsroll, rsyaw, relbow, lspitch, lsroll, lsyaw, lelbow")
    print("  Arms (full): right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw, right_elbow,")
    print("               left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw, left_elbow")
    print("  Torso: waist, torso")
    
    print(f"\n{Fore.YELLOW}Special Commands:{Style.RESET_ALL}")
    print(f"  {Fore.GREEN}reset{Style.RESET_ALL}                    - Reset all joints to position 0 gradually")
    print(f"  {Fore.GREEN}joints{Style.RESET_ALL}                   - Display all joint positions and their limits")
    print(f"  {Fore.GREEN}save{Style.RESET_ALL}                     - Save current joint positions to a file")
    print(f"  {Fore.GREEN}replay <logfile>{Style.RESET_ALL}         - Replay commands from a log file")
    print(f"  {Fore.GREEN}!!{Style.RESET_ALL}                       - Repeat the last command")
    print(f"  {Fore.GREEN}?{Style.RESET_ALL} or {Fore.GREEN}help{Style.RESET_ALL}              - Show this help message")
    print(f"  {Fore.GREEN}quit{Style.RESET_ALL}                     - Exit the program")
    
    print(f"\n{Fore.YELLOW}Tips:{Style.RESET_ALL}")
    print("  - Press TAB to autocomplete commands and joint names")
    print("  - For replay command, TAB will show available log files")
    print("  - In direct control mode, use arrow keys to adjust joint position")
    print("  - Joint positions are limited to prevent damage to the robot")
    print("  - Log files contain a command list at the top for easy replay")
    
    print(f"\n{Fore.YELLOW}Advanced Parameters:{Style.RESET_ALL}")
    print("  <joint> q <value>         - Set joint position (same as <joint> <value>)")
    print("  <joint> dq <value>        - Set joint velocity")
    print("  <joint> tau <value>       - Set joint torque")
    print("  <joint> kp <value>        - Set position gain")
    print("  <joint> kd <value>        - Set velocity gain")
    print("\n")