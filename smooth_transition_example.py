#!/usr/bin/env python3
"""
Example of smooth transitions between pose sequences
Add this logic to your conductor code
"""

# Example: You have two sequences
SEQUENCE_A = [pose1, pose2, pose3, pose4, pose5, pose6, pose7, pose8]
SEQUENCE_B = [pose1_B, pose2_B, pose3_B, pose4_B, pose5_B, pose6_B, pose7_B, pose8_B]

# THE PROBLEM:
# When you switch from SEQUENCE_A to SEQUENCE_B, the last pose of A
# (pose8) doesn't match the first pose of B (pose1_B), causing a jump.

# THE SOLUTION:
# Add a transition period that interpolates from current position to first pose of new sequence

def conduct_with_transitions(self, sequence_schedule):
    """
    Conduct with smooth transitions between sequences

    Args:
        sequence_schedule: List of (sequence, num_measures) tuples
        Example: [
            (SEQUENCE_A, 10),  # Conduct SEQUENCE_A for 10 measures
            (SEQUENCE_B, 8),   # Then SEQUENCE_B for 8 measures
            (SEQUENCE_A, 8),   # Then back to SEQUENCE_A for 8 measures
        ]
    """

    current_sequence = None
    eighth_note_in_sequence = 0

    for seq_idx, (sequence, num_measures) in enumerate(sequence_schedule):
        total_eighths = num_measures * 8  # 8 eighth notes per measure

        # SMOOTH TRANSITION: If switching sequences, interpolate to first pose
        if current_sequence is not None and sequence != current_sequence:
            print(f"Transitioning to new sequence...")
            self.transition_to_sequence(sequence, transition_duration=0.5)  # 0.5 second transition

        current_sequence = sequence

        # Now conduct this sequence
        for eighth_note in range(total_eighths):
            pose_index = eighth_note % 8
            current_pose = sequence[pose_index]
            next_pose = sequence[(pose_index + 1) % 8]

            # Interpolate between poses (your existing code)
            for step in range(self.steps_per_eighth):
                t = step / self.steps_per_eighth
                target_pos = interpolate_pose(current_pose, next_pose, t)

                # Send commands...
                for i, joint in enumerate(self.arm_joints):
                    self.low_cmd.motor_cmd[joint].q = target_pos[i]
                    # ... rest of command setup

                self.arm_sdk_publisher.Write(self.low_cmd)
                time.sleep(self.control_dt)


def transition_to_sequence(self, new_sequence, transition_duration=0.5):
    """
    Smoothly transition from current position to first pose of new sequence

    Args:
        new_sequence: The new pose sequence to transition to
        transition_duration: How long the transition takes (seconds)
    """
    # Get current actual positions from robot state
    current_pos = [self.low_state.motor_state[joint].q for joint in self.arm_joints]

    # Target is the first pose of the new sequence
    target_pose = new_sequence[0]

    # Interpolate smoothly
    steps = int(transition_duration / self.control_dt)

    for step in range(steps):
        t = step / steps

        # Blend from current position to first pose of new sequence
        target_pos = interpolate_pose(
            ConductingPose(current_pos, "current"),
            target_pose,
            t
        )

        # Send commands
        for i, joint in enumerate(self.arm_joints):
            self.low_cmd.motor_cmd[joint].tau = 0.0
            self.low_cmd.motor_cmd[joint].q = target_pos[i]
            self.low_cmd.motor_cmd[joint].dq = 0.0
            self.low_cmd.motor_cmd[joint].kp = self.kp
            self.low_cmd.motor_cmd[joint].kd = self.kd

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)
        time.sleep(self.control_dt)

    print(f"Transition complete! Now conducting new sequence.")


# USAGE EXAMPLE:
"""
# Define your sequences
POSE_SEQUENCE_A = [BEAT_1_DOWN, AND_1_UP, BEAT_2_DOWN, ...]
POSE_SEQUENCE_B = [BEAT_1_DOWN_B, AND_1_UP_B, BEAT_2_DOWN_B, ...]

# Create a schedule
schedule = [
    (POSE_SEQUENCE_A, 10),  # 10 measures of sequence A
    (POSE_SEQUENCE_B, 8),   # 8 measures of sequence B
    (POSE_SEQUENCE_A, 8),   # 8 more measures of sequence A
]

# Conduct with smooth transitions
conductor.conduct_with_transitions(schedule)
"""
