#!/usr/bin/env python3
"""
Test individual joint control using rt/lowcmd channel (low-level control)
This is the proper channel for direct joint position control without coordinated motion
"""

import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC

class G1JointIndex:
    """Joint indices for Unitree G1"""
    # Right arm joints
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28

    # Special joint for arm SDK control flag
    kNotUsedJoint = 29

class LowCmdJointTester:
    def __init__(self):
        """Initialize with rt/lowcmd channel"""
        print("Initializing with rt/lowcmd (low-level joint control)...")

        # Initialize command and state
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.crc = CRC()

        # Create publisher on LOW-LEVEL channel
        self.low_cmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.low_cmd_publisher.Init()

        # Create subscriber for state
        self.low_state_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.low_state_subscriber.Init(self.state_callback, 10)

        # Track robot mode and state
        self.mode_machine = 0
        self.state_received = False

        print("Waiting for first state update...")
        timeout = 5.0
        start_time = time.time()
        while not self.state_received and (time.time() - start_time) < timeout:
            time.sleep(0.1)

        if not self.state_received:
            print("WARNING: No state received after 5 seconds")
        else:
            print(f"State received. mode_machine = {self.mode_machine}")

    def state_callback(self, msg: LowState_):
        """Callback for state updates"""
        self.low_state = msg
        self.mode_machine = msg.mode_machine
        if not self.state_received:
            self.state_received = True

    def get_all_arm_positions(self):
        """Get current positions of all arm joints"""
        joints_to_check = {
            15: "L_ShoulderPitch",
            16: "L_ShoulderRoll",
            17: "L_ShoulderYaw",
            18: "L_Elbow",
            22: "R_ShoulderPitch",
            23: "R_ShoulderRoll",
            24: "R_ShoulderYaw",
            25: "R_Elbow",
            26: "R_WristRoll",
            27: "R_WristPitch",
            28: "R_WristYaw",
        }

        positions = {}
        for idx, name in joints_to_check.items():
            positions[idx] = (name, self.low_state.motor_state[idx].q)
        return positions

    def test_with_gains(self, joint_index, joint_name, kp, kd, movement_amount=0.3):
        """Test a single joint with specified gains"""
        print(f"\n{'='*60}")
        print(f"TESTING: {joint_name} (Joint {joint_index})")
        print(f"Gains: kp={kp}, kd={kd}")
        print(f"{'='*60}\n")

        # Get initial positions
        initial_positions = self.get_all_arm_positions()
        current_pos = initial_positions[joint_index][1]
        target_pos = current_pos + movement_amount

        print("Initial positions:")
        for idx, (name, pos) in sorted(initial_positions.items()):
            marker = " ← TARGET" if idx == joint_index else ""
            print(f"  {name:20} ({idx:2}): {pos:7.3f} rad{marker}")

        print(f"\nCommanding {joint_name}:")
        print(f"  From: {current_pos:.3f} rad")
        print(f"  To:   {target_pos:.3f} rad")
        print(f"  Movement: {movement_amount:+.3f} rad ({movement_amount*57.3:+.1f}°)")

        input(f"\nPress Enter to move with kp={kp}, kd={kd}...")

        # Move for 3 seconds
        duration = 3.0
        start_time = time.time()

        print(f"Moving for {duration} seconds...")
        while (time.time() - start_time) < duration:
            # Zero out all commands
            for i in range(len(self.low_cmd.motor_cmd)):
                self.low_cmd.motor_cmd[i].mode = 0
                self.low_cmd.motor_cmd[i].q = 0.0
                self.low_cmd.motor_cmd[i].dq = 0.0
                self.low_cmd.motor_cmd[i].kp = 0.0
                self.low_cmd.motor_cmd[i].kd = 0.0
                self.low_cmd.motor_cmd[i].tau = 0.0

            # Command ONLY the target joint
            self.low_cmd.motor_cmd[joint_index].mode = 1
            self.low_cmd.motor_cmd[joint_index].q = target_pos
            self.low_cmd.motor_cmd[joint_index].dq = 0.0
            self.low_cmd.motor_cmd[joint_index].kp = kp
            self.low_cmd.motor_cmd[joint_index].kd = kd
            self.low_cmd.motor_cmd[joint_index].tau = 0.0

            # Set mode fields
            self.low_cmd.mode_pr = 0
            self.low_cmd.mode_machine = self.mode_machine

            # Send command
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_cmd_publisher.Write(self.low_cmd)

            time.sleep(0.02)  # 50 Hz

        # Hold position for 1 second
        print("Holding position...")
        hold_start = time.time()
        while (time.time() - hold_start) < 1.0:
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_cmd_publisher.Write(self.low_cmd)
            time.sleep(0.02)

        # Get final positions
        time.sleep(0.1)
        final_positions = self.get_all_arm_positions()

        # Show what actually moved
        print(f"\n{'='*60}")
        print("JOINTS THAT ACTUALLY MOVED:")
        print(f"{'='*60}")

        moved_joints = []
        for idx in sorted(initial_positions.keys()):
            name, initial = initial_positions[idx]
            _, final = final_positions[idx]
            delta = final - initial

            if abs(delta) > 0.01:  # Moved more than 0.01 rad
                marker = " ← COMMANDED" if idx == joint_index else ""
                print(f"  {name:20} ({idx:2}): {initial:7.3f} → {final:7.3f} (Δ {delta:+7.3f}){marker}")
                moved_joints.append((idx, name, delta))

        if not moved_joints:
            print("  *** NO SIGNIFICANT MOVEMENT DETECTED ***")

        # Disable control
        print("\nDisabling control...")
        for i in range(len(self.low_cmd.motor_cmd)):
            self.low_cmd.motor_cmd[i].mode = 0
            self.low_cmd.motor_cmd[i].q = 0.0
            self.low_cmd.motor_cmd[i].kp = 0.0
            self.low_cmd.motor_cmd[i].kd = 0.0
            self.low_cmd.motor_cmd[i].tau = 0.0

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.low_cmd_publisher.Write(self.low_cmd)
        time.sleep(0.5)

        return moved_joints

def main():
    print("="*60)
    print("G1 LOW-LEVEL JOINT CONTROL TEST")
    print("Testing rt/lowcmd channel with different gain values")
    print("="*60)

    # Initialize DDS (ignore return value - may already be initialized)
    ret = ChannelFactoryInitialize(0)
    if ret != 0:
        print(f"Note: ChannelFactoryInitialize returned {ret} (may already be initialized)")
    else:
        print("DDS initialized successfully")

    tester = LowCmdJointTester()

    # Test joint 25 (Right Elbow) with different gains
    test_configs = [
        (10.0, 2.0, "Low gains"),
        (30.0, 5.0, "Medium gains"),
        (60.0, 8.0, "High gains"),
        (100.0, 10.0, "Very high gains"),
    ]

    joint_to_test = G1JointIndex.kRightElbow
    joint_name = "R_Elbow"

    print(f"\nWe will test {joint_name} (Joint {joint_to_test}) with different gains")
    print("to find which gain values give isolated joint control.\n")

    for kp, kd, description in test_configs:
        print(f"\n{'#'*60}")
        print(f"TEST: {description}")
        print(f"{'#'*60}")

        moved = tester.test_with_gains(joint_to_test, joint_name, kp, kd, movement_amount=0.3)

        # Check if only the commanded joint moved
        if len(moved) == 1 and moved[0][0] == joint_to_test:
            print(f"\n✓ SUCCESS! Only {joint_name} moved with kp={kp}, kd={kd}")
        elif len(moved) > 1:
            other_joints = [name for idx, name, delta in moved if idx != joint_to_test]
            print(f"\n⚠ WARNING: Other joints also moved: {', '.join(other_joints)}")
        else:
            print(f"\n✗ FAILED: No movement detected")

        input("\nPress Enter to continue to next gain test...")

    print("\n" + "="*60)
    print("ALL GAIN TESTS COMPLETE!")
    print("="*60)

if __name__ == "__main__":
    main()
