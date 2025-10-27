#!/usr/bin/env python3
"""
Test script for G1 conductor arm positions.

This script allows you to test each conducting position individually
before running the full conducting script. It's useful for:
- Verifying that positions are safe and reachable
- Fine-tuning joint angles for your specific needs
- Checking clearances and motion ranges

Usage:
    python3 test_conductor_positions.py
"""

import time
import sys

# Import Unitree SDK components
try:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC
except ImportError:
    print("Error: Unable to import Unitree SDK. Please ensure unitree_sdk2py is installed.")
    print("Install with: pip install unitree_sdk2py")
    sys.exit(1)


class G1JointIndex:
    """Joint indices for the G1 humanoid robot"""
    # Left leg
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5

    # Right leg
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11

    # Waist
    WaistYaw = 12
    WaistRoll = 13
    WaistA = 13
    WaistPitch = 14
    WaistB = 14

    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21

    # Right arm
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28

    # Special index for arm control enable/disable
    kNotUsedJoint = 29


class PositionTester:
    """
    Test individual conducting positions for the G1 robot.
    """

    def __init__(self):
        """Initialize the position tester."""
        # Control parameters
        self.kp = 60.0  # Proportional gain
        self.kd = 1.5   # Derivative gain
        self.control_dt = 0.02  # Control loop rate: 50 Hz

        # Initialize communication
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.crc = CRC()

        # State tracking
        self.first_state_received = False
        self.mode_machine = 0  # Track robot's control mode

        # Publishers and subscribers
        self.low_cmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.low_cmd_publisher.Init()

        self.low_state_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.low_state_subscriber.Init(self._state_callback, 10)

        # Define all positions (same as in g1_conductor.py)
        self.positions = {
            'rest': {
                'shoulder_pitch': 0.0,
                'shoulder_roll': 0.0,
                'shoulder_yaw': 0.0,
                'elbow': 0.0,
                'wrist_roll': 0.0,
                'wrist_pitch': 0.0,
                'wrist_yaw': 0.0
            },
            'beat1': {  # Down (downbeat)
                'shoulder_pitch': 0.8,
                'shoulder_roll': -0.2,
                'shoulder_yaw': 0.0,
                'elbow': 1.2,
                'wrist_roll': 0.0,
                'wrist_pitch': -0.3,
                'wrist_yaw': 0.0
            },
            'beat2': {  # Left
                'shoulder_pitch': 0.3,
                'shoulder_roll': 0.5,
                'shoulder_yaw': -0.3,
                'elbow': 1.0,
                'wrist_roll': 0.0,
                'wrist_pitch': -0.2,
                'wrist_yaw': 0.0
            },
            'beat3': {  # Right
                'shoulder_pitch': 0.3,
                'shoulder_roll': -0.5,
                'shoulder_yaw': 0.3,
                'elbow': 1.0,
                'wrist_roll': 0.0,
                'wrist_pitch': -0.2,
                'wrist_yaw': 0.0
            },
            'beat4': {  # Up (upbeat)
                'shoulder_pitch': -0.3,
                'shoulder_roll': -0.2,
                'shoulder_yaw': 0.0,
                'elbow': 0.8,
                'wrist_roll': 0.0,
                'wrist_pitch': 0.0,
                'wrist_yaw': 0.0
            }
        }

        print("G1 Position Tester initialized")
        print("=" * 60)

    def _state_callback(self, msg: LowState_):
        """Callback to receive robot state"""
        self.low_state = msg
        self.mode_machine = msg.mode_machine  # Track robot's mode
        if not self.first_state_received:
            self.first_state_received = True

    def _get_current_arm_positions(self):
        """
        Read current arm joint positions from robot state.

        Returns:
            Dictionary with current joint positions
        """
        return {
            'shoulder_pitch': self.low_state.motor_state[G1JointIndex.RightShoulderPitch].q,
            'shoulder_roll': self.low_state.motor_state[G1JointIndex.RightShoulderRoll].q,
            'shoulder_yaw': self.low_state.motor_state[G1JointIndex.RightShoulderYaw].q,
            'elbow': self.low_state.motor_state[G1JointIndex.RightElbow].q,
            'wrist_roll': self.low_state.motor_state[G1JointIndex.RightWristRoll].q,
            'wrist_pitch': self.low_state.motor_state[G1JointIndex.RightWristPitch].q,
            'wrist_yaw': self.low_state.motor_state[G1JointIndex.RightWristYaw].q
        }

    def _interpolate_positions(self, pos1, pos2, ratio):
        """
        Interpolate between two joint position dictionaries.

        Args:
            pos1: Starting position dictionary
            pos2: Target position dictionary
            ratio: Interpolation ratio (0.0 to 1.0)

        Returns:
            Interpolated position dictionary
        """
        result = {}
        for key in pos1.keys():
            result[key] = pos1[key] + (pos2[key] - pos1[key]) * ratio
        return result

    def _set_arm_position(self, position_dict):
        """
        Set the right arm to a specific position.

        Args:
            position_dict: Dictionary with joint positions
        """
        # Enable arm SDK control
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0

        # Right arm joints
        joints = [
            (G1JointIndex.RightShoulderPitch, 'shoulder_pitch'),
            (G1JointIndex.RightShoulderRoll, 'shoulder_roll'),
            (G1JointIndex.RightShoulderYaw, 'shoulder_yaw'),
            (G1JointIndex.RightElbow, 'elbow'),
            (G1JointIndex.RightWristRoll, 'wrist_roll'),
            (G1JointIndex.RightWristPitch, 'wrist_pitch'),
            (G1JointIndex.RightWristYaw, 'wrist_yaw')
        ]

        for joint_idx, joint_name in joints:
            self.low_cmd.motor_cmd[joint_idx].q = position_dict[joint_name]
            self.low_cmd.motor_cmd[joint_idx].dq = 0.0
            self.low_cmd.motor_cmd[joint_idx].kp = self.kp
            self.low_cmd.motor_cmd[joint_idx].kd = self.kd
            self.low_cmd.motor_cmd[joint_idx].tau = 0.0

    def _publish_command(self):
        """Publish the current command with CRC."""
        self.low_cmd.mode_pr = 0  # PR mode
        self.low_cmd.mode_machine = self.mode_machine  # Match robot's mode
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.low_cmd_publisher.Write(self.low_cmd)

    def move_to_position(self, position_name, duration=3.0, hold_time=2.0):
        """
        Smoothly move to a specific position and hold it.

        Args:
            position_name: Name of the position ('rest', 'beat1', 'beat2', 'beat3', 'beat4')
            duration: Time to transition to the position (seconds)
            hold_time: Time to hold the position (seconds)
        """
        if position_name not in self.positions:
            print(f"Error: Unknown position '{position_name}'")
            return False

        # Wait for first state update
        print("Waiting for robot state data...")
        while not self.first_state_received:
            time.sleep(0.1)
        print("Robot state received!")

        target_pos = self.positions[position_name]
        # Read current positions from robot
        start_pos = self._get_current_arm_positions()

        print(f"\nMoving to position: {position_name}")
        print(f"Transition time: {duration}s, Hold time: {hold_time}s")
        print("-" * 60)

        # Display current and target joint angles
        print("\nCurrent joint positions (radians):")
        for joint, angle in start_pos.items():
            print(f"  {joint:20s}: {angle:6.3f} rad ({angle * 57.2958:6.1f}°)")

        print("\nTarget joint angles (radians):")
        for joint, angle in target_pos.items():
            print(f"  {joint:20s}: {angle:6.3f} rad ({angle * 57.2958:6.1f}°)")

        print("\nStarting transition...")

        # Smooth transition to target position
        start_time = time.time()
        while time.time() - start_time < duration:
            ratio = (time.time() - start_time) / duration
            # Use ease-in-out for smooth motion
            if ratio < 0.5:
                ease = 2 * ratio * ratio
            else:
                ease = 1 - 2 * (1 - ratio) * (1 - ratio)

            current_pos = self._interpolate_positions(start_pos, target_pos, ease)
            self._set_arm_position(current_pos)
            self._publish_command()
            time.sleep(self.control_dt)

        # Hold the position
        print(f"Position reached! Holding for {hold_time}s...")
        print("Check that the position looks correct and is safe.")

        hold_start = time.time()
        while time.time() - hold_start < hold_time:
            self._set_arm_position(target_pos)
            self._publish_command()
            time.sleep(self.control_dt)

        print("Done holding position.")
        return True

    def return_to_rest(self, duration=2.0):
        """
        Return to rest position.

        Args:
            duration: Time to transition back to rest (seconds)
        """
        print("\nReturning to rest position...")

        # Read current position from robot
        start_pos = self._get_current_arm_positions()
        target_pos = self.positions['rest']

        start_time = time.time()
        while time.time() - start_time < duration:
            ratio = (time.time() - start_time) / duration
            current_pos = self._interpolate_positions(start_pos, target_pos, ratio)
            self._set_arm_position(current_pos)
            self._publish_command()
            time.sleep(self.control_dt)

        # Disable arm SDK control
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0.0

        self._publish_command()
        print("Returned to rest. Arm control disabled.")

    def test_sequence(self):
        """Test all positions in sequence."""
        print("\n" + "=" * 60)
        print("TESTING ALL POSITIONS IN SEQUENCE")
        print("=" * 60)
        print("\nThis will test each conducting position in order:")
        print("  Rest -> Beat 1 (Down) -> Beat 2 (Left) -> Beat 3 (Right) -> Beat 4 (Up) -> Rest")
        print("\nPress Ctrl+C at any time to stop safely.")

        input("\nPress Enter to start the sequence test...")

        try:
            positions_to_test = ['rest', 'beat1', 'beat2', 'beat3', 'beat4']

            for pos_name in positions_to_test:
                if not self.move_to_position(pos_name, duration=2.0, hold_time=2.0):
                    break
                time.sleep(0.5)  # Brief pause between positions

            print("\n" + "=" * 60)
            print("Sequence test complete!")
            print("=" * 60)

        except KeyboardInterrupt:
            print("\n\nTest interrupted by user")
        finally:
            self.return_to_rest()

    def test_individual_position(self, position_name):
        """Test a single position."""
        print("\n" + "=" * 60)
        print(f"TESTING POSITION: {position_name.upper()}")
        print("=" * 60)

        try:
            if self.move_to_position(position_name, duration=3.0, hold_time=3.0):
                print("\nPosition test complete.")
        except KeyboardInterrupt:
            print("\n\nTest interrupted by user")
        finally:
            self.return_to_rest()

    def show_menu(self):
        """Show interactive menu."""
        while True:
            print("\n" + "=" * 60)
            print("G1 CONDUCTOR POSITION TESTER")
            print("=" * 60)
            print("\nAvailable positions:")
            print("  1. Rest position (neutral)")
            print("  2. Beat 1 - Downbeat (down)")
            print("  3. Beat 2 - Second beat (left)")
            print("  4. Beat 3 - Third beat (right)")
            print("  5. Beat 4 - Upbeat (up)")
            print("\nTest options:")
            print("  6. Test all positions in sequence")
            print("  7. Show position values")
            print("  0. Exit")
            print("=" * 60)

            try:
                choice = input("\nEnter your choice (0-7): ").strip()

                if choice == '0':
                    print("\nExiting tester. Goodbye!")
                    break
                elif choice == '1':
                    self.test_individual_position('rest')
                elif choice == '2':
                    self.test_individual_position('beat1')
                elif choice == '3':
                    self.test_individual_position('beat2')
                elif choice == '4':
                    self.test_individual_position('beat3')
                elif choice == '5':
                    self.test_individual_position('beat4')
                elif choice == '6':
                    self.test_sequence()
                elif choice == '7':
                    self.show_position_values()
                else:
                    print("\nInvalid choice. Please try again.")

            except KeyboardInterrupt:
                print("\n\nInterrupted by user. Exiting...")
                break
            except Exception as e:
                print(f"\nError: {e}")

    def show_position_values(self):
        """Display all position values."""
        print("\n" + "=" * 60)
        print("POSITION VALUES (in radians and degrees)")
        print("=" * 60)

        for pos_name, pos_dict in self.positions.items():
            print(f"\n{pos_name.upper()}:")
            for joint, angle in pos_dict.items():
                degrees = angle * 57.2958
                print(f"  {joint:20s}: {angle:7.3f} rad  ({degrees:7.1f}°)")


def main():
    """Main entry point."""
    print("=" * 60)
    print("G1 CONDUCTOR POSITION TESTER")
    print("=" * 60)
    print("\nThis script allows you to test each conducting position")
    print("individually before running the full conducting script.")
    print("\nIMPORTANT SAFETY NOTES:")
    print("  - Ensure the robot has adequate clearance")
    print("  - Keep emergency stop readily accessible")
    print("  - Monitor the robot at all times")
    print("  - Test with slow movements first")
    print("=" * 60)

    input("\nPress Enter to continue...")

    # Initialize DDS system
    print("\nInitializing DDS communication system...")
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    try:
        tester = PositionTester()
        time.sleep(0.5)  # Brief pause for initialization
        tester.show_menu()
    except KeyboardInterrupt:
        print("\n\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nTest session ended.")


if __name__ == "__main__":
    main()
