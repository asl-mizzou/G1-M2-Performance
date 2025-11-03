#!/usr/bin/env python3
"""
Add these methods to your DualArmConductor class for smooth sequence transitions
"""

def transition_to_sequence_start(self, new_sequence, transition_duration=0.5):
    """
    Smoothly transition from current position to first pose of new sequence
    Call this BEFORE starting a new sequence

    Args:
        new_sequence: List of ConductingPose objects (the new sequence to transition to)
        transition_duration: How long the transition takes in seconds (default 0.5)
    """
    print(f"Transitioning to: {new_sequence[0].name}")

    # Get current actual positions from robot state
    current_pos = [self.low_state.motor_state[joint].q for joint in self.arm_joints]

    # Target is the first pose of the new sequence
    target_pose = new_sequence[0]

    # Calculate number of steps for smooth interpolation
    steps = int(transition_duration / self.control_dt)

    # Enable arm SDK
    self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0

    # Smoothly interpolate from current position to first pose
    for step in range(steps):
        t = step / steps

        # Blend from current position to first pose of new sequence
        target_pos = interpolate_pose(
            ConductingPose(current_pos, "current"),
            target_pose,
            t
        )

        # Send commands to all joints
        for i, joint in enumerate(self.arm_joints):
            self.low_cmd.motor_cmd[joint].tau = 0.0
            self.low_cmd.motor_cmd[joint].q = target_pos[i]
            self.low_cmd.motor_cmd[joint].dq = 0.0
            self.low_cmd.motor_cmd[joint].kp = self.kp
            self.low_cmd.motor_cmd[joint].kd = self.kd

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)
        time.sleep(self.control_dt)

    print(f"Transition complete! Starting new sequence.")


def conduct_with_sequence_changes(self, sequence_schedule):
    """
    Conduct with smooth transitions between different pose sequences

    Args:
        sequence_schedule: List of (sequence, num_measures) tuples
        Example:
            [
                (POSE_SEQUENCE_A, 10),  # Sequence A for 10 measures
                (POSE_SEQUENCE_B, 8),   # Sequence B for 8 measures
                (POSE_SEQUENCE_A, 8),   # Back to A for 8 measures
            ]
    """
    print("\n" + "="*60)
    print(f"🎵 CONDUCTING WITH SEQUENCE CHANGES at {self.bpm} BPM 🎵")
    print("="*60)
    print(f"Beat duration: {self.beat_duration:.3f}s")
    print(f"Eighth note duration: {self.eighth_note_duration:.3f}s")
    print("="*60 + "\n")

    self.conducting = True
    overall_measure = 1
    overall_beat = 1

    for seg_idx, (sequence, num_measures) in enumerate(sequence_schedule):
        print(f"\n{'='*60}")
        print(f"SEGMENT {seg_idx + 1}: {num_measures} measures")
        print(f"{'='*60}")

        # SMOOTH TRANSITION: Interpolate to first pose of this sequence
        if seg_idx > 0:  # Skip transition for first sequence (already in ready position)
            self.transition_to_sequence_start(sequence, transition_duration=0.5)

        # Conduct this sequence for the specified number of measures
        total_eighths = num_measures * 8  # 8 eighth notes per measure

        for eighth_note in range(total_eighths):
            pose_index = eighth_note % 8
            current_pose = sequence[pose_index]
            next_pose = sequence[(pose_index + 1) % 8]

            # Update measure and beat display
            if eighth_note % 2 == 0:  # On beat (not "and")
                overall_beat = (eighth_note // 2) % 4 + 1
                if overall_beat == 1 and eighth_note > 0:
                    overall_measure += 1
                print(f"Measure {overall_measure} | Beat {overall_beat}")

            # Interpolate between current and next pose
            for step in range(self.steps_per_eighth):
                t = step / self.steps_per_eighth
                target_pos = interpolate_pose(current_pose, next_pose, t)

                # Send commands to all joints
                for i, joint in enumerate(self.arm_joints):
                    self.low_cmd.motor_cmd[joint].tau = 0.0
                    self.low_cmd.motor_cmd[joint].q = target_pos[i]
                    self.low_cmd.motor_cmd[joint].dq = 0.0
                    self.low_cmd.motor_cmd[joint].kp = self.kp
                    self.low_cmd.motor_cmd[joint].kd = self.kd

                self.low_cmd.crc = self.crc.Crc(self.low_cmd)
                self.arm_sdk_publisher.Write(self.low_cmd)
                time.sleep(self.control_dt)

    print(f"\n✓ All segments complete!")
    self.conducting = False


# EXAMPLE USAGE:
"""
# In your main conducting mode function, add a new option:

def conduct_mode_with_changes(bpm=68):
    '''Conducting with rhythm changes'''

    # Initialize
    conductor = DualArmConductor(bpm=bpm)
    conductor.Init()

    # Move to ready position (first pose of first sequence)
    conductor.move_to_pose(POSE_SEQUENCE_A[0], duration=3.0)

    # Define the sequence schedule
    schedule = [
        (POSE_SEQUENCE_A, 10),  # 10 measures of sequence A
        (POSE_SEQUENCE_B, 8),   # 8 measures of sequence B
        (POSE_SEQUENCE_A, 8),   # 8 measures back to A
    ]

    # Conduct with smooth transitions
    conductor.conduct_with_sequence_changes(schedule)

    # Clean up
    conductor.return_to_neutral()
    conductor.disable_control()
"""
