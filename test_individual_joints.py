#!/usr/bin/env python3
"""
Test each right arm joint individually to see what actually moves.
"""

import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

class JointTester:
    def __init__(self):
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.crc = CRC()
        self.state_received = False
        self.mode_machine = 0

        self.low_cmd_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.low_cmd_publisher.Init()

        self.low_state_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.low_state_subscriber.Init(self._state_callback, 10)

    def _state_callback(self, msg):
        self.low_state = msg
        self.mode_machine = msg.mode_machine
        if not self.state_received:
            self.state_received = True

    def test_single_joint(self, joint_index, joint_name, movement_amount=0.2):
        """Test moving a single joint"""
        print("\n" + "="*60)
        print(f"TESTING JOINT {joint_index}: {joint_name}")
        print("="*60)

        # Wait for state
        while not self.state_received:
            time.sleep(0.1)

        # Show ALL joint positions before
        print("\nALL joint positions BEFORE:")
        all_joints = list(range(15, 29))  # Both arms
        initial_positions = {}
        for j in all_joints:
            initial_positions[j] = self.low_state.motor_state[j].q
            joint_label = f"Joint {j}"
            if j == 15: joint_label = "L_ShoulderPitch"
            elif j == 16: joint_label = "L_ShoulderRoll"
            elif j == 17: joint_label = "L_ShoulderYaw"
            elif j == 18: joint_label = "L_Elbow"
            elif j == 22: joint_label = "R_ShoulderPitch"
            elif j == 23: joint_label = "R_ShoulderRoll"
            elif j == 24: joint_label = "R_ShoulderYaw"
            elif j == 25: joint_label = "R_Elbow"
            elif j == 26: joint_label = "R_WristRoll"
            elif j == 27: joint_label = "R_WristPitch"
            elif j == 28: joint_label = "R_WristYaw"
            print(f"  {joint_label:20s} ({j:2d}): {initial_positions[j]:7.3f} rad")

        initial = self.low_state.motor_state[joint_index].q
        target = initial + movement_amount

        print(f"\nCommanding joint {joint_index} ({joint_name}):")
        print(f"  From: {initial:.3f} rad")
        print(f"  To:   {target:.3f} rad")
        print(f"  Movement: {movement_amount:+.3f} rad ({movement_amount*57.3:+.1f}°)")

        input(f"\nPress Enter to move joint {joint_index} ({joint_name})...")

        # Move over 3 seconds
        duration = 3.0
        start_time = time.time()

        print(f"Moving for {duration} seconds...")

        while time.time() - start_time < duration:
            ratio = (time.time() - start_time) / duration
            current_measured = self.low_state.motor_state[joint_index].q
            current_target = current_measured + (target - current_measured) * ratio

            # Enable arm SDK
            self.low_cmd.motor_cmd[29].q = 1.0

            # Command ONLY this joint
            self.low_cmd.motor_cmd[joint_index].mode = 1
            self.low_cmd.motor_cmd[joint_index].q = current_target
            self.low_cmd.motor_cmd[joint_index].dq = 0.0
            self.low_cmd.motor_cmd[joint_index].kp = 60.0
            self.low_cmd.motor_cmd[joint_index].kd = 1.5
            self.low_cmd.motor_cmd[joint_index].tau = 0.0

            self.low_cmd.mode_pr = 0
            self.low_cmd.mode_machine = self.mode_machine
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_cmd_publisher.Write(self.low_cmd)

            time.sleep(0.02)

        # Hold for 2 seconds
        print("Holding position...")
        hold_start = time.time()
        while time.time() - hold_start < 2.0:
            self.low_cmd.motor_cmd[29].q = 1.0
            self.low_cmd.motor_cmd[joint_index].mode = 1
            self.low_cmd.motor_cmd[joint_index].q = target
            self.low_cmd.motor_cmd[joint_index].dq = 0.0
            self.low_cmd.motor_cmd[joint_index].kp = 60.0
            self.low_cmd.motor_cmd[joint_index].kd = 1.5
            self.low_cmd.motor_cmd[joint_index].tau = 0.0

            self.low_cmd.mode_pr = 0
            self.low_cmd.mode_machine = self.mode_machine
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_cmd_publisher.Write(self.low_cmd)
            time.sleep(0.02)

        # Check which joints moved
        print("\n" + "="*60)
        print("JOINTS THAT ACTUALLY MOVED:")
        print("="*60)
        final_positions = {}
        for j in all_joints:
            final_positions[j] = self.low_state.motor_state[j].q
            delta = final_positions[j] - initial_positions[j]
            if abs(delta) > 0.01:
                joint_label = f"Joint {j}"
                if j == 15: joint_label = "L_ShoulderPitch"
                elif j == 16: joint_label = "L_ShoulderRoll"
                elif j == 17: joint_label = "L_ShoulderYaw"
                elif j == 18: joint_label = "L_Elbow"
                elif j == 22: joint_label = "R_ShoulderPitch"
                elif j == 23: joint_label = "R_ShoulderRoll"
                elif j == 24: joint_label = "R_ShoulderYaw"
                elif j == 25: joint_label = "R_Elbow"
                elif j == 26: joint_label = "R_WristRoll"
                elif j == 27: joint_label = "R_WristPitch"
                elif j == 28: joint_label = "R_WristYaw"

                marker = " ← COMMANDED" if j == joint_index else ""
                print(f"  {joint_label:20s} ({j:2d}): {initial_positions[j]:7.3f} → {final_positions[j]:7.3f} (Δ {delta:+7.3f}){marker}")

        # Disable
        print("\nDisabling arm control...")
        for i in range(25):
            self.low_cmd.motor_cmd[29].q = 0.0
            self.low_cmd.mode_pr = 0
            self.low_cmd.mode_machine = self.mode_machine
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_cmd_publisher.Write(self.low_cmd)
            time.sleep(0.02)

        input("\nPress Enter to continue to next joint...")

def main():
    print("="*60)
    print("INDIVIDUAL JOINT MOVEMENT TEST")
    print("="*60)
    print("\nThis will test each right arm joint individually.")
    print("For each joint, you'll see:")
    print("  1. All current positions")
    print("  2. The commanded joint will move +0.2 rad")
    print("  3. Which joints ACTUALLY moved")
    print("\nThis will reveal the true joint mapping!")
    print("="*60)

    input("\nPress Enter to start...")

    # Initialize DDS
    print("\nInitializing DDS...")
    ChannelFactoryInitialize(0)

    tester = JointTester()
    time.sleep(0.5)

    # Test each right arm joint
    joints_to_test = [
        (22, "R_ShoulderPitch"),
        (23, "R_ShoulderRoll"),
        (24, "R_ShoulderYaw"),
        (25, "R_Elbow"),
        (26, "R_WristRoll"),
        (27, "R_WristPitch"),
        (28, "R_WristYaw"),
    ]

    try:
        for joint_idx, joint_name in joints_to_test:
            tester.test_single_joint(joint_idx, joint_name, movement_amount=0.2)

        print("\n" + "="*60)
        print("ALL TESTS COMPLETE!")
        print("="*60)

    except KeyboardInterrupt:
        print("\n\nCancelled by user")

if __name__ == "__main__":
    main()
