#!/usr/bin/env python3
"""
Test individual conducting positions - Based on working dab example
Uses dual-arm control via rt/arm_sdk channel
"""

import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

kPi = 3.141592654
kPi_2 = 1.57079632

class G1JointIndex:
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

    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14

    kNotUsedJoint = 29

class ConductorTester:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.02
        self.duration_ = 2.0  # 2 second transition

        self.kp = 60.0  # From dab example
        self.kd = 1.5   # From dab example

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update_low_state = False
        self.crc = CRC()

        self.target_position = None
        self.start_position = None
        self.moving = False
        self.hold_duration = 2.0
        self.hold_start_time = None

        # All arm joints
        self.arm_joints = [
            G1JointIndex.LeftShoulderPitch,  G1JointIndex.LeftShoulderRoll,
            G1JointIndex.LeftShoulderYaw,    G1JointIndex.LeftElbow,
            G1JointIndex.LeftWristRoll,      G1JointIndex.LeftWristPitch,
            G1JointIndex.LeftWristYaw,
            G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
            G1JointIndex.RightShoulderYaw,   G1JointIndex.RightElbow,
            G1JointIndex.RightWristRoll,     G1JointIndex.RightWristPitch,
            G1JointIndex.RightWristYaw,
            G1JointIndex.WaistYaw,
            G1JointIndex.WaistRoll,
            G1JointIndex.WaistPitch
        ]

        # Define positions (same as conductor script)
        self.ready_pos = [
            0., 0., 0., 0., 0., 0., 0.,  # Left arm: neutral
            -kPi/3, -kPi/6, 0., kPi/3, 0., 0., 0.,  # Right arm: ready
            0., 0., 0.   # Waist
        ]

        self.beat1_pos = [
            0., 0., 0., 0., 0., 0., 0.,  # Left arm: neutral
            0., -kPi/6, 0., kPi/3, 0., 0., 0.,  # Right arm: down
            0., 0., 0.
        ]

        self.beat2_pos = [
            0., 0., 0., 0., 0., 0., 0.,  # Left arm: neutral
            -kPi/4, -kPi/6, kPi/6, kPi/3, 0., 0., 0.,  # Right arm: left
            0., 0., 0.
        ]

        self.beat3_pos = [
            0., 0., 0., 0., 0., 0., 0.,  # Left arm: neutral
            -kPi/4, -kPi/6, -kPi/6, kPi/3, 0., 0., 0.,  # Right arm: right
            0., 0., 0.
        ]

        self.beat4_pos = [
            0., 0., 0., 0., 0., 0., 0.,  # Left arm: neutral
            -kPi/2, -kPi/6, 0., kPi/3, 0., 0., 0.,  # Right arm: up
            0., 0., 0.
        ]

        # Rest position - both arms down at sides
        self.rest_pos = [
            0., 0., 0., 0., 0., 0., 0.,  # Left arm: at side
            0., 0., 0., 0., 0., 0., 0.,  # Right arm: at side
            0., 0., 0.  # Waist: neutral
        ]

        self.positions = {
            'ready': self.ready_pos,
            'beat1': self.beat1_pos,
            'beat2': self.beat2_pos,
            'beat3': self.beat3_pos,
            'beat4': self.beat4_pos,
            'rest': self.rest_pos
        }

    def Init(self):
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )

        while self.first_update_low_state == False:
            time.sleep(0.1)

        if self.first_update_low_state == True:
            self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        if self.first_update_low_state == False:
            self.first_update_low_state = True

    def move_to_position(self, position_name):
        """Start moving to a position"""
        if position_name not in self.positions:
            print(f"Unknown position: {position_name}")
            return False

        self.target_position = self.positions[position_name]
        self.start_position = [self.low_state.motor_state[joint].q for joint in self.arm_joints]
        self.time_ = 0.0
        self.moving = True
        self.hold_start_time = None
        print(f"Moving to {position_name}...")
        return True

    def disable_arm_control(self):
        """Disable arm SDK control and zero out commands"""
        print("Disabling arm control...")

        # Gradually release control
        for step in range(20):  # 20 steps = 0.4 seconds
            ratio = step / 20.0

            # Fade out arm SDK enable flag
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0 - ratio

            # Keep commanding current position with reducing gains
            for i, joint in enumerate(self.arm_joints):
                self.low_cmd.motor_cmd[joint].tau = 0.
                self.low_cmd.motor_cmd[joint].q = self.target_position[i] if self.target_position else 0.
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp * (1.0 - ratio)
                self.low_cmd.motor_cmd[joint].kd = self.kd * (1.0 - ratio)

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)
            time.sleep(0.02)

        # Final command with everything at zero
        for i in range(len(self.low_cmd.motor_cmd)):
            self.low_cmd.motor_cmd[i].tau = 0.
            self.low_cmd.motor_cmd[i].q = 0.
            self.low_cmd.motor_cmd[i].dq = 0.
            self.low_cmd.motor_cmd[i].kp = 0.
            self.low_cmd.motor_cmd[i].kd = 0.

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)
        print("Arm control disabled.")

    def LowCmdWrite(self):
        """Control loop"""
        if not self.moving:
            return

        self.time_ += self.control_dt_

        # Enable arm SDK
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1

        if self.time_ < self.duration_:
            # Interpolate to target
            ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)

            for i, joint in enumerate(self.arm_joints):
                current_q = ratio * self.target_position[i] + (1.0 - ratio) * self.start_position[i]
                self.low_cmd.motor_cmd[joint].tau = 0.
                self.low_cmd.motor_cmd[joint].q = current_q
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd

        else:
            # Hold at target position
            if self.hold_start_time is None:
                self.hold_start_time = time.time()
                print("Position reached. Holding...")

            for i, joint in enumerate(self.arm_joints):
                self.low_cmd.motor_cmd[joint].tau = 0.
                self.low_cmd.motor_cmd[joint].q = self.target_position[i]
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd

            # Check if hold time expired
            if time.time() - self.hold_start_time > self.hold_duration:
                self.moving = False
                print("Hold complete.")

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)

def print_menu():
    print("\n" + "="*60)
    print("CONDUCTING POSITION TESTER")
    print("="*60)
    print("Choose a position to test:")
    print("  0. Ready position (arm raised, ready to conduct)")
    print("  1. Beat 1 - Down (downbeat)")
    print("  2. Beat 2 - Left")
    print("  3. Beat 3 - Right")
    print("  4. Beat 4 - Up (upbeat)")
    print("  r. Rest position (arms at sides)")
    print("  q. Quit (returns to rest and releases control)")
    print("="*60)

def main():
    print("Initializing conductor position tester...")

    # Initialize DDS
    ret = ChannelFactoryInitialize(0)
    if ret != 0:
        print(f"Note: ChannelFactoryInitialize returned {ret}")

    tester = ConductorTester()
    tester.Init()
    tester.Start()

    print("Tester ready!")

    position_map = {
        '0': 'ready',
        '1': 'beat1',
        '2': 'beat2',
        '3': 'beat3',
        '4': 'beat4',
        'r': 'rest'
    }

    try:
        while True:
            print_menu()
            choice = input("\nEnter choice: ").strip().lower()

            if choice == 'q':
                print("\nReturning arms to rest position...")
                tester.move_to_position('rest')

                # Wait for movement to complete
                while tester.moving:
                    time.sleep(0.1)

                # Disable arm control
                tester.disable_arm_control()
                print("Exiting...")
                break

            if choice in position_map:
                tester.move_to_position(position_map[choice])

                # Wait for movement to complete
                while tester.moving:
                    time.sleep(0.1)
            else:
                print("Invalid choice!")

    except KeyboardInterrupt:
        print("\n\nInterrupted! Returning arms to rest position...")
        tester.move_to_position('rest')

        # Wait for movement to complete
        while tester.moving:
            time.sleep(0.1)

        # Disable arm control
        tester.disable_arm_control()
        print("Exiting...")

if __name__ == '__main__':
    main()
