#!/usr/bin/env python3
"""
Simple script to move right arm to rest position (arms by side).
"""

import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

class ArmRest:
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

    def move_to_rest(self):
        """Move right arm to rest position (straight down by side)"""
        print("Moving right arm to rest position...")

        # Wait for state
        while not self.state_received:
            time.sleep(0.1)

        # Rest position: all joints at 0.0 (straight down)
        rest_position = {
            22: 0.0,  # ShoulderPitch
            23: 0.0,  # ShoulderRoll
            24: 0.0,  # ShoulderYaw
            25: 0.0,  # Elbow
            26: 0.0,  # WristRoll
            27: 0.0,  # WristPitch
            28: 0.0,  # WristYaw
        }

        # Read current positions
        current = {}
        for joint in rest_position.keys():
            current[joint] = self.low_state.motor_state[joint].q

        print("\nCurrent positions:")
        for joint, pos in current.items():
            print(f"  Joint {joint}: {pos:.3f} rad")

        print("\nTarget rest positions:")
        for joint, pos in rest_position.items():
            print(f"  Joint {joint}: {pos:.3f} rad")

        # Smooth movement over 5 seconds
        duration = 5.0
        start_time = time.time()

        print(f"\nMoving to rest position over {duration} seconds...")

        while time.time() - start_time < duration:
            ratio = (time.time() - start_time) / duration

            # Enable arm SDK
            self.low_cmd.motor_cmd[29].q = 1.0

            # Command all right arm joints
            for joint in rest_position.keys():
                current_measured = self.low_state.motor_state[joint].q
                target = current_measured + (rest_position[joint] - current_measured) * ratio

                self.low_cmd.motor_cmd[joint].mode = 1
                self.low_cmd.motor_cmd[joint].q = target
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].kp = 60.0
                self.low_cmd.motor_cmd[joint].kd = 1.5
                self.low_cmd.motor_cmd[joint].tau = 0.0

            # Set mode fields
            self.low_cmd.mode_pr = 0
            self.low_cmd.mode_machine = self.mode_machine
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_cmd_publisher.Write(self.low_cmd)

            time.sleep(0.02)

        print("\n✓ Rest position reached!")

        # Show final positions
        print("\nFinal positions:")
        for joint in rest_position.keys():
            final = self.low_state.motor_state[joint].q
            print(f"  Joint {joint}: {final:.3f} rad")

        # Hold position for 3 seconds
        print("\nHolding position for 3 seconds...")
        hold_start = time.time()
        while time.time() - hold_start < 3.0:
            self.low_cmd.motor_cmd[29].q = 1.0
            for joint in rest_position.keys():
                self.low_cmd.motor_cmd[joint].mode = 1
                self.low_cmd.motor_cmd[joint].q = rest_position[joint]
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].kp = 60.0
                self.low_cmd.motor_cmd[joint].kd = 1.5
                self.low_cmd.motor_cmd[joint].tau = 0.0

            self.low_cmd.mode_pr = 0
            self.low_cmd.mode_machine = self.mode_machine
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_cmd_publisher.Write(self.low_cmd)
            time.sleep(0.02)

        # Disable
        print("\nDisabling arm control...")
        for i in range(25):
            self.low_cmd.motor_cmd[29].q = 0.0
            self.low_cmd.mode_pr = 0
            self.low_cmd.mode_machine = self.mode_machine
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_cmd_publisher.Write(self.low_cmd)
            time.sleep(0.02)

        print("✓ Done!")

def main():
    print("="*60)
    print("MOVE RIGHT ARM TO REST POSITION")
    print("="*60)
    print("\nThis will move the right arm to rest position:")
    print("  - All joints at 0.0 rad")
    print("  - Arm straight down by the side")
    print("\nPress Ctrl+C to cancel")
    print("="*60)

    input("\nPress Enter to start...")

    # Initialize DDS
    print("\nInitializing DDS...")
    ChannelFactoryInitialize(0)

    arm = ArmRest()
    time.sleep(0.5)

    try:
        arm.move_to_rest()
    except KeyboardInterrupt:
        print("\n\nCancelled by user")

if __name__ == "__main__":
    main()
