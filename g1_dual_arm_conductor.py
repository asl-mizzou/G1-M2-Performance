#!/usr/bin/env python3
"""
G1 Dual-Arm Conductor - Converted from C++ original
Performs synchronized dual-arm conducting motions for marching band
Includes testing mode to review and adjust positions
"""

import time
import sys
import numpy as np
from dataclasses import dataclass
from typing import List

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

@dataclass
class ConductingPose:
    """A conducting pose with 17 joint angles"""
    joint_angles: List[float]  # 17 values: left arm (7), right arm (7), waist (3)
    name: str

    def __post_init__(self):
        assert len(self.joint_angles) == 17, f"Expected 17 joint angles, got {len(self.joint_angles)}"

# USER-TESTED POSES - From original C++ code
# Order: LeftShoulder(Pitch,Roll,Yaw), LeftElbow, LeftWrist(Roll,Pitch,Yaw),
#        RightShoulder(Pitch,Roll,Yaw), RightElbow, RightWrist(Roll,Pitch,Yaw),
#        Waist(Yaw,Roll,Pitch)

BEAT_1_DOWN = ConductingPose(
    [0.40, 0.25, 0.00, -0.40, 1.50, 0.00, 0.00,
     0.40, -0.25, 0.00, -0.40, -1.50, 0.00, 0.00,
     0.00, 0.00, 0.00],
    "Beat 1 - Downbeat"
)

AND_1_UP = ConductingPose(
    [0.40, 0.25, 0.00, -0.80, 1.50, 0.00, 0.80,
     0.40, -0.25, 0.00, -0.80, -1.50, 0.00, -0.80,
     0.00, 0.00, 0.00],
    "And 1 - Up"
)
'''
[0.40, 0.25, -0.30, -0.35, 1.50, 0.00, 0.60,
     0.40, -0.25, 0.30, -0.35, -1.50, 0.00, -0.60,
     0.00, 0.00, 0.00],
'''

BEAT_2_DOWN = ConductingPose(
    [0.40, 0.25, 0.00, -0.40, 1.50, 0.00, 0.60,
     0.40, -0.25, 0.00, -0.40, -1.50, 0.00, -0.60,
     0.00, 0.00, 0.00],
    "Beat 2 - Down"
)

AND_2_CENTER = ConductingPose(
    [0.40, 0.25, -0.40, -0.95, 1.20, 0.00, 0.40,
     0.40, -0.25, 0.40, -0.95, -1.20, 0.00, -0.40,
     0.00, 0.00, 0.00],
    "And 2 - Center"
)

BEAT_3_DOWN = ConductingPose(
    [0.40, 0.25, 0.00, -0.40, 1.50, 0.00, 0.60,
     0.40, -0.25, 0.00, -0.40, -1.50, 0.00, -0.60,
     0.00, 0.00, 0.00],
    "Beat 3 - Down"
)

AND_3_WIDE = ConductingPose(
    [0.40, 0.85, 0.80, -0.30, 0.80, 0.00, 0.90,
     0.40, -0.85, -0.80, -0.30, -0.80, 0.00, -0.90,
     0.00, 0.00, 0.00],
    "And 3 - Wide"
)

BEAT_4_DOWN = ConductingPose(
    [0.40, 0.25, 0.00, -0.40, 1.50, 0.00, 0.00,
     0.40, -0.25, 0.00, -0.40, -1.50, 0.00, 0.00,
     0.00, 0.00, 0.00],
    "Beat 4 - Down"
)

AND_4_PREP = ConductingPose(
    [0.40, 0.25, 0.00, -0.80, 1.50, 0.00, 0.80,
     0.40, -0.25, 0.00, -0.80, -1.50, 0.00, -0.80,
     0.00, 0.00, 0.00],
    "And 4 - Prep"
)

NEUTRAL_POSE = ConductingPose(
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0],
    "Neutral (Rest)"
)

##LOW Tempo Pose Sequesce
LT_BEAT_1_DOWN = ConductingPose(
    [0.40, 0.25, 0.00, -0.40, 1.50, 0.00, 0.60,
     0.40, -0.25, 0.00, -0.40, -1.50, 0.00, -0.60,
     0.00, 0.00, 0.00],
    "Low Tempo Beat 1 - Down"
)

LT_AND_1_CENTER = ConductingPose(
    [0.40, 0.25, -0.40, -0.95, 1.20, 0.00, 0.40,
     0.40, -0.25, 0.40, -0.95, -1.20, 0.00, -0.40,
     0.00, 0.00, 0.00],
    "Low Tempo Beat 1 Center- Up"
)

LT_BEAT_2_DOWN = ConductingPose(
    [0.40, 0.25, 0.00, -0.40, 1.50, 0.00, 0.60,
     0.40, -0.25, 0.00, -0.40, -1.50, 0.00, -0.60,
     0.00, 0.00, 0.00],
    "Low Tempo Beat 2 - Down"
)

LT_AND_2_UP = ConductingPose(
    [0.40, 0.25, 0.00, -0.80, 1.50, 0.00, 0.80,
     0.40, -0.25, 0.00, -0.80, -1.50, 0.00, -0.80,
     0.00, 0.00, 0.00],
    "And 2 - Up"
)



# Sequence of poses for one complete measure (8 eighth notes)
POSE_SEQUENCE = [
    BEAT_1_DOWN,    # Beat 1
    AND_1_UP,       # and
    BEAT_2_DOWN,    # Beat 2
    AND_2_CENTER,   # and
    BEAT_3_DOWN,    # Beat 3
    AND_3_WIDE,     # and
    BEAT_4_DOWN,    # Beat 4
    AND_4_PREP      # and
]

# LOW TEMPO Sequence of poses for one complete measure of low tempo(8 eighth notes)
LT_POSE_SEQUENCE = [
    LT_BEAT_1_DOWN,
    LT_BEAT_1_DOWN,
    LT_AND_1_CENTER,
    LT_AND_1_CENTER,
    LT_BEAT_2_DOWN,
    LT_BEAT_2_DOWN,
    LT_AND_2_UP,
    LT_AND_2_UP
]

def interpolate_pose(start: ConductingPose, end: ConductingPose, t: float) -> List[float]:
    """Linear interpolation between two poses"""
    t = np.clip(t, 0.0, 1.0)
    return [start.joint_angles[i] + t * (end.joint_angles[i] - start.joint_angles[i])
            for i in range(17)]

class DualArmConductor:
    def __init__(self, bpm=68, num_measures=26):
        self.bpm = bpm
        self.num_measures = num_measures
        self.beat_duration = 60.0 / bpm
        self.eighth_note_duration = self.beat_duration / 2.0
        self.control_dt = 0.02  # 50 Hz (like dab example)

        self.steps_per_eighth = int(self.eighth_note_duration / self.control_dt)
        self.total_beats = num_measures * 4
        self.total_eighths = self.total_beats * 2

        # Control parameters from C++ code
        self.kp = 60.0
        self.kd = 1.5

        # Communication
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update_low_state = False
        self.crc = CRC()

        # Joint indices (17 total)
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

        # State
        self.conducting = False
        self.current_target = None

    def Init(self):
        """Initialize publishers and subscribers"""
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

        print("Waiting for first state update...")
        timeout = 5.0
        start_time = time.time()
        while not self.first_update_low_state and (time.time() - start_time) < timeout:
            time.sleep(0.1)

        if not self.first_update_low_state:
            print("WARNING: No state received after 5 seconds")
        else:
            print("State received!")

    def LowStateHandler(self, msg: LowState_):
        """Handle state updates"""
        self.low_state = msg
        if not self.first_update_low_state:
            self.first_update_low_state = True

    def move_to_pose(self, pose: ConductingPose, duration=3.0):
        """Move to a specific pose over the given duration"""
        print(f"Moving to: {pose.name}")

        # Get current positions
        current_pos = [self.low_state.motor_state[joint].q for joint in self.arm_joints]

        steps = int(duration / self.control_dt)

        # Enable arm SDK
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0

        for step in range(steps):
            t = step / steps
            target_pos = interpolate_pose(
                ConductingPose(current_pos, "current"),
                pose,
                t
            )

            for i, joint in enumerate(self.arm_joints):
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].q = target_pos[i]
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)
            time.sleep(self.control_dt)

        # Hold at final position
        for _ in range(10):
            for i, joint in enumerate(self.arm_joints):
                self.low_cmd.motor_cmd[joint].q = pose.joint_angles[i]

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)
            time.sleep(self.control_dt)

        print(f"Reached: {pose.name}")
        self.current_target = pose

    def conduct_measures(self):
        """Execute the full conducting sequence"""
        print("\n" + "="*60)
        print(f"🎵 CONDUCTING {self.num_measures} MEASURES at {self.bpm} BPM 🎵")
        print("="*60)
        print(f"Beat duration: {self.beat_duration:.3f}s")
        print(f"Eighth note duration: {self.eighth_note_duration:.3f}s")
        print("="*60 + "\n")

        self.conducting = True
        current_measure = 1
        current_beat = 1

        for eighth_note in range(self.total_eighths):
            pose_index = eighth_note % 8

            #add transitions to conducting speed
            if current_measure <= 4:
                input("Press Enter to Continue")
                current_pose = POSE_SEQUENCE[pose_index]
                next_pose = POSE_SEQUENCE[(pose_index + 1) % 8]
            elif current_measure > 4:
                input("Press Enter to Continue")
                current_pose = LT_POSE_SEQUENCE[pose_index]
                next_pose = LT_POSE_SEQUENCE[(pose_index + 1) % 8]

            # Update measure and beat display
            if eighth_note % 2 == 0:  # On beat (not "and")
                current_beat = (eighth_note // 2) % 4 + 1
                if current_beat == 1 and eighth_note > 0:
                    current_measure += 1
                print(f"Measure {current_measure} | Beat {current_beat}")

            # Interpolate between current and next pose
            for step in range(self.steps_per_eighth):
                t = step / self.steps_per_eighth
                target_pos = interpolate_pose(current_pose, next_pose, t)

                for i, joint in enumerate(self.arm_joints):
                    self.low_cmd.motor_cmd[joint].tau = 0.0
                    self.low_cmd.motor_cmd[joint].q = target_pos[i]
                    self.low_cmd.motor_cmd[joint].dq = 0.0
                    self.low_cmd.motor_cmd[joint].kp = self.kp
                    self.low_cmd.motor_cmd[joint].kd = self.kd

                self.low_cmd.crc = self.crc.Crc(self.low_cmd)
                self.arm_sdk_publisher.Write(self.low_cmd)
                time.sleep(self.control_dt)

        print(f"\n✓ Conducting complete! {self.num_measures} measures finished.")
        self.conducting = False

    def return_to_neutral(self):
        """Return arms to neutral/rest position"""
        print("Returning to neutral position...")
        self.move_to_pose(NEUTRAL_POSE, duration=3.0)

    def disable_control(self):
        """Gradually disable arm control"""
        print("Disabling arm control...")

        # Gradually reduce gains
        for step in range(20):
            ratio = step / 20.0

            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0 - ratio

            for i, joint in enumerate(self.arm_joints):
                self.low_cmd.motor_cmd[joint].kp = self.kp * (1.0 - ratio)
                self.low_cmd.motor_cmd[joint].kd = self.kd * (1.0 - ratio)

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)
            time.sleep(0.02)

        # Zero everything
        for i in range(len(self.low_cmd.motor_cmd)):
            self.low_cmd.motor_cmd[i].tau = 0.0
            self.low_cmd.motor_cmd[i].q = 0.0
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = 0.0
            self.low_cmd.motor_cmd[i].kd = 0.0

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)
        print("Control disabled.")

def print_testing_menu():
    """Print the testing menu"""
    print("\n" + "="*60)
    print("G1 DUAL-ARM CONDUCTING - TESTING MODE")
    print("="*60)
    print("Test individual poses:")
    print("  1. Beat 1 - Downbeat")
    print("  2. And 1 - Up")
    print("  3. Beat 2 - Down")
    print("  4. And 2 - Center")
    print("  5. Beat 3 - Down")
    print("  6. And 3 - Wide")
    print("  7. Beat 4 - Down")
    print("  8. And 4 - Prep")
    print()
    print("  s. Sequence through all 8 poses (slow)")
    print("  n. Return to Neutral (rest)")
    print("  q. Quit (returns to neutral and releases control)")
    print("="*60)

def test_mode():
    """Interactive testing mode"""
    print("\n" + "="*60)
    print("INITIALIZING TESTING MODE")
    print("="*60)

    # Initialize DDS
    ret = ChannelFactoryInitialize(0)
    if ret != 0:
        print(f"Note: ChannelFactoryInitialize returned {ret}")

    conductor = DualArmConductor()
    conductor.Init()

    # Move to ready position
    print("\nMoving to Beat 1 ready position...")
    conductor.move_to_pose(BEAT_1_DOWN, duration=3.0)

    pose_map = {
        '1': BEAT_1_DOWN,
        '2': AND_1_UP,
        '3': BEAT_2_DOWN,
        '4': AND_2_CENTER,
        '5': BEAT_3_DOWN,
        '6': AND_3_WIDE,
        '7': BEAT_4_DOWN,
        '8': AND_4_PREP,
        'n': NEUTRAL_POSE
    }

    try:
        while True:
            print_testing_menu()
            choice = input("\nEnter choice: ").strip().lower()

            if choice == 'q':
                print("\nReturning to neutral and releasing control...")
                conductor.return_to_neutral()
                conductor.disable_control()
                print("Exiting testing mode.")
                break

            elif choice == 's':
                print("\nSequencing through all 8 poses...")
                for i, pose in enumerate(POSE_SEQUENCE, 1):
                    print(f"\n[{i}/8] {pose.name}")
                    conductor.move_to_pose(pose, duration=2.0)
                    time.sleep(1.0)
                print("\nSequence complete!")

            elif choice in pose_map:
                conductor.move_to_pose(pose_map[choice], duration=2.0)

            else:
                print("Invalid choice!")

    except KeyboardInterrupt:
        print("\n\nInterrupted! Returning to neutral...")
        conductor.return_to_neutral()
        conductor.disable_control()

def conduct_mode(bpm=68, num_measures=26):
    """Full conducting mode"""
    print("\n" + "="*60)
    print("G1 DUAL-ARM CONDUCTING - PERFORMANCE MODE")
    print("="*60)
    print(f"Tempo: {bpm} BPM")
    print(f"Measures: {num_measures}")
    print("="*60)
    print("\nWARNING: Ensure clear space around robot!")
    print("Press Ctrl+C to stop anytime.")
    print()
    input("Press ENTER to initialize...")

    # Initialize DDS
    ret = ChannelFactoryInitialize(0)
    if ret != 0:
        print(f"Note: ChannelFactoryInitialize returned {ret}")

    conductor = DualArmConductor(bpm=bpm, num_measures=num_measures)
    conductor.Init()

    try:
        # Move to ready position
        print("\nMoving to ready position...")
        conductor.move_to_pose(BEAT_1_DOWN, duration=3.0)

        input(f"\nPress ENTER to start conducting {num_measures} measures...")

        # Conduct!
        conductor.conduct_measures()

        # Return to neutral
        conductor.return_to_neutral()
        conductor.disable_control()

        print("\n🎉 Performance complete! 🎉")

    except KeyboardInterrupt:
        print("\n\nInterrupted! Stopping...")
        conductor.return_to_neutral()
        conductor.disable_control()

def print_main_menu():
    """Print main menu"""
    print("\n" + "="*60)
    print("G1 DUAL-ARM CONDUCTING SYSTEM")
    print("="*60)
    print("Choose mode:")
    print("  1. Testing Mode (review and adjust poses)")
    print("  2. Conducting Mode (full performance)")
    print("  q. Quit")
    print("="*60)

def main():
    """Main entry point"""
    print("="*60)
    print("G1 Drum Major Conducting Program")
    print("Converted from C++ original")
    print("="*60)

    while True:
        print_main_menu()
        choice = input("\nEnter choice: ").strip().lower()

        if choice == 'q':
            print("Exiting program.")
            break

        elif choice == '1':
            test_mode()

        elif choice == '2':
            bpm = input("Enter BPM (default 68): ").strip()
            bpm = int(bpm) if bpm else 68

            measures = input("Enter number of measures (default 26): ").strip()
            measures = int(measures) if measures else 26

            conduct_mode(bpm=bpm, num_measures=measures)

        else:
            print("Invalid choice!")

if __name__ == '__main__':
    main()
