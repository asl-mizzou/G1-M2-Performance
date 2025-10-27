#!/usr/bin/env python3
"""
G1 Conducting Script - Based on working dab example
Uses dual-arm control via rt/arm_sdk channel
Left arm stays in neutral position, right arm conducts
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

    kNotUsedJoint = 29  # Arm SDK enable flag

class G1Conductor:
    def __init__(self, bpm=120):
        self.time_ = 0.0
        self.control_dt_ = 0.02  # 50 Hz control loop
        self.bpm = bpm
        self.beat_duration = 60.0 / bpm  # Duration of one beat in seconds
        self.transition_duration = self.beat_duration * 0.8  # Use 80% of beat for transition

        self.kp = 60.0  # From dab example
        self.kd = 1.5   # From dab example

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update_low_state = False
        self.crc = CRC()

        self.current_beat = 0  # 0=ready, 1=down, 2=left, 3=right, 4=up
        self.stage = "init"  # init, ready, conducting, done

        # All arm joints (left + right + waist)
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

        # LEFT ARM: Keep in neutral/rest position (all zeros means arms at sides)
        # RIGHT ARM: Conducting positions
        # Order: L_ShPitch, L_ShRoll, L_ShYaw, L_Elbow, L_WrRoll, L_WrPitch, L_WrYaw,
        #        R_ShPitch, R_ShRoll, R_ShYaw, R_Elbow, R_WrRoll, R_WrPitch, R_WrYaw,
        #        WaistYaw, WaistRoll, WaistPitch

        # Ready position: Right arm raised, ready to conduct
        self.ready_pos = [
            0., 0., 0., 0., 0., 0., 0.,  # Left arm: neutral at side
            -kPi/3,      # R_ShoulderPitch: raised up (~60°)
            -kPi/6,      # R_ShoulderRoll: slightly out from body (~30°)
            0.,          # R_ShoulderYaw: neutral
            kPi/3,       # R_Elbow: bent (~60°)
            0.,          # R_WristRoll: neutral
            0.,          # R_WristPitch: neutral
            0.,          # R_WristYaw: neutral
            0., 0., 0.   # Waist: neutral
        ]

        # Beat 1: DOWN (downbeat)
        self.beat1_pos = [
            0., 0., 0., 0., 0., 0., 0.,  # Left arm: neutral
            0.,          # R_ShoulderPitch: down to neutral
            -kPi/6,      # R_ShoulderRoll: maintain
            0.,          # R_ShoulderYaw: neutral
            kPi/3,       # R_Elbow: maintain bend
            0., 0., 0.,  # Wrist: neutral
            0., 0., 0.   # Waist: neutral
        ]

        # Beat 2: LEFT
        self.beat2_pos = [
            0., 0., 0., 0., 0., 0., 0.,  # Left arm: neutral
            -kPi/4,      # R_ShoulderPitch: slightly raised
            -kPi/6,      # R_ShoulderRoll: maintain
            kPi/6,       # R_ShoulderYaw: rotate left (~30°)
            kPi/3,       # R_Elbow: maintain
            0., 0., 0.,  # Wrist: neutral
            0., 0., 0.   # Waist: neutral
        ]

        # Beat 3: RIGHT
        self.beat3_pos = [
            0., 0., 0., 0., 0., 0., 0.,  # Left arm: neutral
            -kPi/4,      # R_ShoulderPitch: slightly raised
            -kPi/6,      # R_ShoulderRoll: maintain
            -kPi/6,      # R_ShoulderYaw: rotate right (~-30°)
            kPi/3,       # R_Elbow: maintain
            0., 0., 0.,  # Wrist: neutral
            0., 0., 0.   # Waist: neutral
        ]

        # Beat 4: UP (upbeat)
        self.beat4_pos = [
            0., 0., 0., 0., 0., 0., 0.,  # Left arm: neutral
            -kPi/2,      # R_ShoulderPitch: raised high (~90°)
            -kPi/6,      # R_ShoulderRoll: maintain
            0.,          # R_ShoulderYaw: neutral
            kPi/3,       # R_Elbow: maintain
            0., 0., 0.,  # Wrist: neutral
            0., 0., 0.   # Waist: neutral
        ]

        self.beat_positions = [
            self.ready_pos,
            self.beat1_pos,
            self.beat2_pos,
            self.beat3_pos,
            self.beat4_pos
        ]

        self.beat_names = ["Ready", "Beat 1 (Down)", "Beat 2 (Left)", "Beat 3 (Right)", "Beat 4 (Up)"]

    def Init(self):
        """Initialize publishers and subscribers"""
        # Create publisher on rt/arm_sdk channel (like dab example)
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()

        # Create subscriber for state
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        """Start the control loop"""
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )

        print("Waiting for first state update...")
        while self.first_update_low_state == False:
            time.sleep(0.1)

        if self.first_update_low_state == True:
            print("State received. Starting control loop...")
            self.stage_start_time = time.time()
            self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        """Handle state updates"""
        self.low_state = msg
        if self.first_update_low_state == False:
            self.first_update_low_state = True

    def get_interpolated_position(self, start_pos, end_pos, ratio):
        """Interpolate between two positions"""
        ratio = np.clip(ratio, 0.0, 1.0)
        return [ratio * end + (1.0 - ratio) * start for start, end in zip(end_pos, start_pos)]

    def LowCmdWrite(self):
        """Main control loop - called at 50 Hz"""
        self.time_ += self.control_dt_
        elapsed = time.time() - self.stage_start_time

        # Enable arm SDK control
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1

        if self.stage == "init":
            # Move to ready position
            if elapsed < 2.0:  # 2 second transition to ready
                ratio = elapsed / 2.0
                current_positions = [self.low_state.motor_state[joint].q for joint in self.arm_joints]
                target_positions = self.get_interpolated_position(current_positions, self.ready_pos, ratio)

                for i, joint in enumerate(self.arm_joints):
                    self.low_cmd.motor_cmd[joint].tau = 0.
                    self.low_cmd.motor_cmd[joint].q = target_positions[i]
                    self.low_cmd.motor_cmd[joint].dq = 0.
                    self.low_cmd.motor_cmd[joint].kp = self.kp
                    self.low_cmd.motor_cmd[joint].kd = self.kd
            else:
                print(f"Ready to conduct at {self.bpm} BPM!")
                print("Starting conducting pattern...")
                self.stage = "conducting"
                self.stage_start_time = time.time()
                self.current_beat = 1  # Start with beat 1 (down)

        elif self.stage == "conducting":
            # Conducting loop through beats 1-4
            beat_elapsed = elapsed % self.beat_duration

            if beat_elapsed < self.transition_duration:
                # Transition to next beat
                ratio = beat_elapsed / self.transition_duration
                prev_beat = self.current_beat - 1 if self.current_beat > 0 else 4
                current_positions = self.beat_positions[prev_beat]
                target_positions = self.get_interpolated_position(
                    current_positions,
                    self.beat_positions[self.current_beat],
                    ratio
                )

                for i, joint in enumerate(self.arm_joints):
                    self.low_cmd.motor_cmd[joint].tau = 0.
                    self.low_cmd.motor_cmd[joint].q = target_positions[i]
                    self.low_cmd.motor_cmd[joint].dq = 0.
                    self.low_cmd.motor_cmd[joint].kp = self.kp
                    self.low_cmd.motor_cmd[joint].kd = self.kd
            else:
                # Hold at beat position
                for i, joint in enumerate(self.arm_joints):
                    self.low_cmd.motor_cmd[joint].tau = 0.
                    self.low_cmd.motor_cmd[joint].q = self.beat_positions[self.current_beat][i]
                    self.low_cmd.motor_cmd[joint].dq = 0.
                    self.low_cmd.motor_cmd[joint].kp = self.kp
                    self.low_cmd.motor_cmd[joint].kd = self.kd

            # Check if we need to move to next beat
            if beat_elapsed < self.control_dt_:  # Just started a new beat
                print(f"  {self.beat_names[self.current_beat]}")
                self.current_beat += 1
                if self.current_beat > 4:
                    self.current_beat = 1  # Loop back to beat 1

        # Publish command
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)

def main():
    if len(sys.argv) > 1:
        try:
            bpm = int(sys.argv[1])
        except:
            print("Usage: python3 g1_conductor_dual_arm.py [BPM]")
            print("Example: python3 g1_conductor_dual_arm.py 120")
            return
    else:
        bpm = 120  # Default tempo

    print("="*60)
    print("G1 ROBOT CONDUCTOR - 4/4 Time")
    print("="*60)
    print(f"Tempo: {bpm} BPM")
    print(f"Beat duration: {60.0/bpm:.2f} seconds")
    print()
    print("WARNING: Ensure no obstacles around the robot!")
    print("The robot will conduct with its right arm.")
    print("Press Ctrl+C to stop.")
    print()
    input("Press Enter to start conducting...")

    # Initialize DDS
    if len(sys.argv) > 2:
        ChannelFactoryInitialize(0, sys.argv[2])
    else:
        ret = ChannelFactoryInitialize(0)
        if ret != 0:
            print(f"Note: ChannelFactoryInitialize returned {ret}")

    conductor = G1Conductor(bpm=bpm)
    conductor.Init()
    conductor.Start()

    # Run until interrupted
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nStopping conductor...")
        sys.exit(0)

if __name__ == '__main__':
    main()
