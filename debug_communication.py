#!/usr/bin/env python3
"""
Debug script to verify DDS communication with G1 robot.
This will help diagnose why commands aren't working.
"""

import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

class DebugTester:
    def __init__(self):
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.crc = CRC()
        self.state_count = 0
        self.mode_machine = 0  # Track robot's mode

        # Publishers and subscribers
        self.low_cmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.low_cmd_publisher.Init()

        self.low_state_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.low_state_subscriber.Init(self._state_callback, 10)

        print("Communication channels initialized")

    def _state_callback(self, msg: LowState_):
        """Callback to receive robot state"""
        self.low_state = msg
        self.state_count += 1
        # Always update mode_machine from robot state
        self.mode_machine = msg.mode_machine

        if self.state_count == 1:
            print(f"✓ First state message received!")
            print(f"  mode_machine: {msg.mode_machine}")
            print(f"  mode_pr: {msg.mode_pr}")

            # Print right arm joint positions
            print(f"\nRight arm current positions:")
            print(f"  ShoulderPitch (22): {msg.motor_state[22].q:.3f} rad")
            print(f"  ShoulderRoll  (23): {msg.motor_state[23].q:.3f} rad")
            print(f"  ShoulderYaw   (24): {msg.motor_state[24].q:.3f} rad")
            print(f"  Elbow         (25): {msg.motor_state[25].q:.3f} rad")
            print(f"  WristRoll     (26): {msg.motor_state[26].q:.3f} rad")
            print(f"  WristPitch    (27): {msg.motor_state[27].q:.3f} rad")
            print(f"  WristYaw      (28): {msg.motor_state[28].q:.3f} rad")

    def test_enable_flag(self):
        """Test setting the arm enable flag"""
        print("\n" + "="*60)
        print("TEST: Setting arm enable flag (motor_cmd[29].q = 1.0)")
        print("="*60)

        # Wait for state
        print("Waiting for state data...")
        while self.state_count == 0:
            time.sleep(0.1)

        # Set enable flag
        print(f"\nSending enable command with mode_machine={self.mode_machine}...")
        for i in range(50):  # Send for 1 second (50 * 0.02s)
            self.low_cmd.motor_cmd[29].q = 1.0
            self.low_cmd.mode_pr = 0  # Mode.PR
            self.low_cmd.mode_machine = self.mode_machine
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_cmd_publisher.Write(self.low_cmd)
            time.sleep(0.02)

        print("✓ Enable flag sent 50 times")

    def test_simple_movement(self):
        """Test commanding a simple arm movement"""
        print("\n" + "="*60)
        print("TEST: Commanding simple elbow movement")
        print("="*60)

        # Read current elbow position
        initial_elbow = self.low_state.motor_state[25].q
        target_elbow = initial_elbow + 0.3  # Move 0.3 radians (~17 degrees)

        print(f"Initial elbow position: {initial_elbow:.3f} rad")
        print(f"Target elbow position:  {target_elbow:.3f} rad")
        print(f"Movement: {target_elbow - initial_elbow:.3f} rad (~{(target_elbow - initial_elbow)*57.3:.1f} degrees)")

        print("\nSending movement commands for 3 seconds...")
        start_time = time.time()
        duration = 3.0

        while time.time() - start_time < duration:
            ratio = (time.time() - start_time) / duration

            # Read CURRENT measured position each iteration
            current_measured = self.low_state.motor_state[25].q

            # Interpolate from current measured position to target
            current_target = current_measured + (target_elbow - current_measured) * ratio

            # Enable arm SDK
            self.low_cmd.motor_cmd[29].q = 1.0

            # Command elbow joint
            self.low_cmd.motor_cmd[25].mode = 1  # Enable this joint
            self.low_cmd.motor_cmd[25].q = current_target
            self.low_cmd.motor_cmd[25].dq = 0.0
            self.low_cmd.motor_cmd[25].kp = 60.0
            self.low_cmd.motor_cmd[25].kd = 1.5
            self.low_cmd.motor_cmd[25].tau = 0.0

            # Set mode fields
            self.low_cmd.mode_pr = 0  # Mode.PR
            self.low_cmd.mode_machine = self.mode_machine

            # Send command
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_cmd_publisher.Write(self.low_cmd)

            time.sleep(0.02)

        # Check final position
        final_elbow = self.low_state.motor_state[25].q
        print(f"\n✓ Movement complete")
        print(f"Final elbow position: {final_elbow:.3f} rad")
        print(f"Expected: {target_elbow:.3f} rad")
        print(f"Error: {abs(final_elbow - target_elbow):.3f} rad")

        if abs(final_elbow - target_elbow) < 0.1:
            print("✓ Movement SUCCESSFUL!")
        else:
            print("✗ Movement FAILED - joint didn't move as expected")

        # Disable
        print("\nDisabling arm control...")
        for i in range(25):
            self.low_cmd.motor_cmd[29].q = 0.0
            self.low_cmd.mode_pr = 0
            self.low_cmd.mode_machine = self.mode_machine
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_cmd_publisher.Write(self.low_cmd)
            time.sleep(0.02)

    def test_all_joints_movement(self):
        """Test commanding ALL arm joints (hold others, move elbow)"""
        print("\n" + "="*60)
        print("TEST: Commanding ALL arm joints together")
        print("="*60)

        # Define all right arm joints
        arm_joints = [22, 23, 24, 25, 26, 27, 28]  # All 7 right arm joints

        # Read initial positions
        initial_positions = {}
        for joint in arm_joints:
            initial_positions[joint] = self.low_state.motor_state[joint].q

        # Set elbow target
        target_elbow = initial_positions[25] + 0.3

        print(f"Initial elbow position: {initial_positions[25]:.3f} rad")
        print(f"Target elbow position:  {target_elbow:.3f} rad")
        print("Other joints will hold their current positions")

        print("\nSending movement commands for 3 seconds...")
        start_time = time.time()
        duration = 3.0

        while time.time() - start_time < duration:
            ratio = (time.time() - start_time) / duration

            # Enable arm SDK
            self.low_cmd.motor_cmd[29].q = 1.0

            # Command all arm joints
            for joint in arm_joints:
                self.low_cmd.motor_cmd[joint].mode = 1

                if joint == 25:  # Elbow - move it
                    current_measured = self.low_state.motor_state[joint].q
                    target = current_measured + (target_elbow - current_measured) * ratio
                else:  # Other joints - hold position
                    target = self.low_state.motor_state[joint].q

                self.low_cmd.motor_cmd[joint].q = target
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].kp = 60.0
                self.low_cmd.motor_cmd[joint].kd = 1.5
                self.low_cmd.motor_cmd[joint].tau = 0.0

            # Set mode fields
            self.low_cmd.mode_pr = 0
            self.low_cmd.mode_machine = self.mode_machine

            # Send command
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_cmd_publisher.Write(self.low_cmd)

            time.sleep(0.02)

        # Check final position
        final_elbow = self.low_state.motor_state[25].q
        print(f"\n✓ Movement complete")
        print(f"Final elbow position: {final_elbow:.3f} rad")
        print(f"Expected: {target_elbow:.3f} rad")
        print(f"Error: {abs(final_elbow - target_elbow):.3f} rad")

        if abs(final_elbow - target_elbow) < 0.1:
            print("✓ Movement SUCCESSFUL!")
        else:
            print("✗ Movement FAILED - joint didn't move as expected")

        # Disable
        print("\nDisabling arm control...")
        for i in range(25):
            self.low_cmd.motor_cmd[29].q = 0.0
            self.low_cmd.mode_pr = 0
            self.low_cmd.mode_machine = self.mode_machine
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_cmd_publisher.Write(self.low_cmd)
            time.sleep(0.02)

    def run(self):
        """Run all tests"""
        try:
            self.test_enable_flag()
            time.sleep(1)
            self.test_simple_movement()
            time.sleep(1)
            self.test_all_joints_movement()

            print("\n" + "="*60)
            print("All tests complete!")
            print("="*60)

        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
        finally:
            # Disable arm control
            self.low_cmd.motor_cmd[29].q = 0.0
            self.low_cmd.mode_pr = 0
            self.low_cmd.mode_machine = self.mode_machine
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_cmd_publisher.Write(self.low_cmd)

def main():
    print("="*60)
    print("G1 COMMUNICATION DEBUG TOOL")
    print("="*60)
    print("\nThis script will:")
    print("1. Check if we can receive robot state")
    print("2. Test sending the arm enable flag")
    print("3. Test a simple elbow movement")
    print("\nPress Ctrl+C at any time to stop")
    print("="*60)

    input("\nPress Enter to start...")

    # Initialize DDS
    print("\nInitializing DDS...")
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    tester = DebugTester()
    time.sleep(0.5)
    tester.run()

if __name__ == "__main__":
    main()
