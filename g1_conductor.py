#!/usr/bin/env python3
"""
Unitree G1 Music Conductor
Controls the G1 humanoid robot to conduct music in 4/4 time.

This script makes the robot conduct using its right arm in the standard
4/4 conducting pattern:
  Beat 1: Down (downbeat)
  Beat 2: Left
  Beat 3: Right
  Beat 4: Up (upbeat)

Usage:
    python3 g1_conductor.py [--bpm TEMPO] [--measures COUNT]

Arguments:
    --bpm TEMPO       Tempo in beats per minute (default: 120)
    --measures COUNT  Number of measures to conduct (default: infinite)
"""

import time
import sys
import math
import argparse
import numpy as np

# Import Unitree SDK components
try:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC
    from unitree_sdk2py.utils.thread import RecurrentThread
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


class G1Conductor:
    """
    Controls the G1 robot to conduct music in 4/4 time.
    """

    def __init__(self, bpm=120, measures=None):
        """
        Initialize the conductor.

        Args:
            bpm: Tempo in beats per minute (default: 120)
            measures: Number of measures to conduct (None = infinite)
        """
        self.bpm = bpm
        self.measures = measures
        self.beat_duration = 60.0 / bpm  # Duration of one beat in seconds

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

        # Publishers and subscribers
        self.low_cmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.low_cmd_publisher.Init()

        self.low_state_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.low_state_subscriber.Init(self._state_callback, 10)

        # Control state
        self.running = False
        self.start_time = None
        self.initial_positions = None

        # Define conducting positions for right arm joints
        # Positions are in radians: [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, WristRoll, WristPitch, WristYaw]
        self.rest_position = {
            'shoulder_pitch': 0.0,
            'shoulder_roll': 0.0,
            'shoulder_yaw': 0.0,
            'elbow': 0.0,
            'wrist_roll': 0.0,
            'wrist_pitch': 0.0,
            'wrist_yaw': 0.0
        }

        # Define the 4/4 conducting pattern positions
        # Beat 1: Down (downbeat)
        self.beat1_position = {
            'shoulder_pitch': 0.8,   # Forward/down
            'shoulder_roll': -0.2,   # Slight inward
            'shoulder_yaw': 0.0,
            'elbow': 1.2,            # Bent elbow
            'wrist_roll': 0.0,
            'wrist_pitch': -0.3,     # Slight wrist angle
            'wrist_yaw': 0.0
        }

        # Beat 2: Left
        self.beat2_position = {
            'shoulder_pitch': 0.3,   # Medium height
            'shoulder_roll': 0.5,    # Out to the left
            'shoulder_yaw': -0.3,
            'elbow': 1.0,
            'wrist_roll': 0.0,
            'wrist_pitch': -0.2,
            'wrist_yaw': 0.0
        }

        # Beat 3: Right
        self.beat3_position = {
            'shoulder_pitch': 0.3,   # Medium height
            'shoulder_roll': -0.5,   # Out to the right
            'shoulder_yaw': 0.3,
            'elbow': 1.0,
            'wrist_roll': 0.0,
            'wrist_pitch': -0.2,
            'wrist_yaw': 0.0
        }

        # Beat 4: Up (upbeat)
        self.beat4_position = {
            'shoulder_pitch': -0.3,  # Up
            'shoulder_roll': -0.2,   # Slight inward
            'shoulder_yaw': 0.0,
            'elbow': 0.8,
            'wrist_roll': 0.0,
            'wrist_pitch': 0.0,
            'wrist_yaw': 0.0
        }

        # Store beat positions in order
        self.beat_positions = [
            self.beat1_position,
            self.beat2_position,
            self.beat3_position,
            self.beat4_position
        ]

        print(f"G1 Conductor initialized")
        print(f"Tempo: {self.bpm} BPM")
        print(f"Beat duration: {self.beat_duration:.3f} seconds")
        if measures:
            print(f"Measures to conduct: {measures}")
        else:
            print("Conducting indefinitely (Ctrl+C to stop)")

    def _state_callback(self, msg: LowState_):
        """Callback to receive robot state"""
        self.low_state = msg
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

    def _get_beat_position(self, elapsed_time):
        """
        Calculate the target arm position based on elapsed time.

        Args:
            elapsed_time: Time since conducting started (seconds)

        Returns:
            Target position dictionary
        """
        # Calculate which beat we're on
        beat_phase = (elapsed_time % (4 * self.beat_duration)) / self.beat_duration
        current_beat = int(beat_phase)  # 0, 1, 2, or 3
        beat_progress = beat_phase - current_beat  # 0.0 to 1.0 within the beat

        # Get current and next beat positions
        current_pos = self.beat_positions[current_beat]
        next_pos = self.beat_positions[(current_beat + 1) % 4]

        # Use smooth easing for more natural conducting motion
        # Ease-in-out cubic function for smooth acceleration/deceleration
        if beat_progress < 0.5:
            # Accelerate into the beat
            t = beat_progress * 2
            ease = t * t * t / 2
        else:
            # Decelerate after the beat
            t = (beat_progress - 0.5) * 2
            ease = 0.5 + (1 - (1 - t) * (1 - t) * (1 - t)) / 2

        # Interpolate between positions
        return self._interpolate_positions(current_pos, next_pos, ease)

    def _control_loop(self):
        """Main control loop running at 50 Hz"""
        if not self.running:
            return

        elapsed_time = time.time() - self.start_time

        # Check if we should stop (if measure limit is set)
        if self.measures is not None:
            measure_duration = 4 * self.beat_duration  # 4 beats per measure
            if elapsed_time >= self.measures * measure_duration:
                print(f"\nCompleted {self.measures} measures. Returning to rest position...")
                self.stop()
                return

        # Get target position for current time
        target_position = self._get_beat_position(elapsed_time)

        # Set arm to target position
        self._set_arm_position(target_position)

        # Calculate CRC and publish command
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.low_cmd_publisher.Write(self.low_cmd)

    def start(self):
        """Start conducting"""
        print("\nPreparing to conduct...")

        # Wait for first state update
        print("Waiting for robot state data...")
        while not self.first_state_received:
            time.sleep(0.1)
        print("Robot state received!")

        print("Moving to starting position...")

        # Read current position and move to beat4 (upbeat starting position)
        current_pos = self._get_current_arm_positions()
        self.start_time = time.time()
        init_duration = 2.0

        while time.time() - self.start_time < init_duration:
            ratio = (time.time() - self.start_time) / init_duration
            target_pos = self._interpolate_positions(current_pos, self.beat4_position, ratio)
            self._set_arm_position(target_pos)
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_cmd_publisher.Write(self.low_cmd)
            time.sleep(self.control_dt)

        # Start conducting
        print("Conducting started!")
        print("Press Ctrl+C to stop")

        self.running = True
        self.start_time = time.time()

        # Start control thread
        self.control_thread = RecurrentThread(
            interval=self.control_dt,
            target=self._control_loop,
            name="conductor_control"
        )
        self.control_thread.Start()

    def stop(self):
        """Stop conducting and return to rest position"""
        if not self.running:
            return

        self.running = False

        # Stop control thread
        if hasattr(self, 'control_thread'):
            self.control_thread.Stop()

        print("Returning to rest position...")

        # Smoothly return to rest position
        return_duration = 2.0
        start_time = time.time()

        # Read current position from robot
        current_pos = self._get_current_arm_positions()

        while time.time() - start_time < return_duration:
            ratio = (time.time() - start_time) / return_duration
            target_pos = self._interpolate_positions(current_pos, self.rest_position, ratio)
            self._set_arm_position(target_pos)
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.low_cmd_publisher.Write(self.low_cmd)
            time.sleep(self.control_dt)

        # Disable arm SDK control
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0.0

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.low_cmd_publisher.Write(self.low_cmd)

        print("Conducting stopped.")

    def run(self):
        """Main run method"""
        try:
            self.start()

            # Keep running until interrupted or measure limit reached
            while self.running:
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n\nInterrupted by user")

        finally:
            self.stop()


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='Control Unitree G1 robot to conduct music in 4/4 time',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument(
        '--bpm',
        type=int,
        default=120,
        help='Tempo in beats per minute (default: 120)'
    )

    parser.add_argument(
        '--measures',
        type=int,
        default=None,
        help='Number of measures to conduct (default: infinite)'
    )

    args = parser.parse_args()

    # Validate arguments
    if args.bpm < 40 or args.bpm > 240:
        print("Error: BPM must be between 40 and 240")
        sys.exit(1)

    if args.measures is not None and args.measures < 1:
        print("Error: Measures must be at least 1")
        sys.exit(1)

    # Initialize DDS system
    print("Initializing DDS communication system...")
    if len(sys.argv) > 1 and not sys.argv[1].startswith('--'):
        # First arg is network interface (not a flag)
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    # Create and run conductor
    conductor = G1Conductor(bpm=args.bpm, measures=args.measures)
    conductor.run()


if __name__ == "__main__":
    main()
