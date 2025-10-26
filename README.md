# G1 Robot Music Conductor

This project enables a Unitree G1 humanoid robot to conduct music in 4/4 time signature, perfect for marching band performances at football games!

## Overview

The `g1_conductor.py` script controls the G1's right arm to perform the standard conducting pattern for 4/4 time:

- **Beat 1**: Downbeat (straight down)
- **Beat 2**: Left motion
- **Beat 3**: Right motion
- **Beat 4**: Upbeat (straight up)

The movements are smooth and natural, using cubic easing functions for acceleration and deceleration at each beat.

## Requirements

### Hardware
- Unitree G1 humanoid robot
- Network connection to the robot

### Software
- Python 3.7 or higher
- Unitree SDK2 Python package

## Installation

1. Install the Unitree SDK2 Python package:
```bash
pip install unitree_sdk2py
```

2. Ensure your computer is connected to the G1 robot's network

3. Make the script executable (optional):
```bash
chmod +x g1_conductor.py
```

## Usage

### Basic Usage

Run with default settings (120 BPM, continuous conducting):
```bash
python3 g1_conductor.py
```

### Custom Tempo

Conduct at a specific tempo (e.g., 100 BPM for a slower march):
```bash
python3 g1_conductor.py --bpm 100
```

### Limited Duration

Conduct for a specific number of measures (e.g., 32 measures):
```bash
python3 g1_conductor.py --bpm 120 --measures 32
```

### Command-Line Options

- `--bpm TEMPO`: Set the tempo in beats per minute (40-240, default: 120)
- `--measures COUNT`: Number of measures to conduct (default: continuous)

### Examples

Common marching band tempos:

```bash
# Slow march (76-90 BPM)
python3 g1_conductor.py --bpm 80

# Standard march (120 BPM)
python3 g1_conductor.py --bpm 120

# Quick march (140 BPM)
python3 g1_conductor.py --bpm 140

# Fight song tempo (160-180 BPM)
python3 g1_conductor.py --bpm 160
```

## How It Works

### Conducting Pattern

The script implements the traditional 4/4 conducting pattern using the robot's right arm:

1. **Initialization**: The arm smoothly transitions from rest to the starting position (upbeat)
2. **Conducting Loop**: The arm moves through the four-beat pattern continuously
3. **Timing**: Each beat is precisely timed based on the specified BPM
4. **Easing**: Smooth cubic easing creates natural acceleration and deceleration
5. **Shutdown**: When stopped, the arm returns smoothly to rest position

### Joint Control

The script controls 8 joints in the right arm:
- Shoulder Pitch (up/down)
- Shoulder Roll (side to side)
- Shoulder Yaw (rotation)
- Elbow Pitch (bend)
- Elbow Roll (rotation)
- Wrist Yaw (rotation)
- Wrist Roll (rotation)
- Wrist Pitch (up/down)

### Control Parameters

- **Update Rate**: 50 Hz (0.02s intervals)
- **Position Control**: PD controller with kp=60.0, kd=1.5
- **Communication**: DDS (Data Distribution Service) protocol

## Safety Considerations

1. **Clear Space**: Ensure the robot has adequate clearance for arm movements
2. **Emergency Stop**: Be ready to use the robot's emergency stop if needed
3. **Supervision**: Always supervise the robot during operation
4. **Testing**: Test with slower tempos first before using at performance speed

## Stopping the Conductor

Press `Ctrl+C` at any time to safely stop conducting. The robot will:
1. Stop the conducting motion
2. Smoothly return the arm to rest position
3. Disable arm control

## Troubleshooting

### Connection Issues
- Verify network connection to the G1 robot
- Check that the robot is powered on and ready
- Ensure no other programs are controlling the robot

### Import Errors
```bash
# Reinstall the SDK
pip install --upgrade unitree_sdk2py
```

### Jerky Movements
- Lower the tempo to ensure smooth motion
- Check that the robot is on stable ground
- Verify sufficient battery charge

## Customization

You can customize the conducting pattern by modifying the beat position dictionaries in the `G1Conductor` class:

- `beat1_position`: Downbeat position
- `beat2_position`: Left beat position
- `beat3_position`: Right beat position
- `beat4_position`: Upbeat position

Each position is defined by joint angles in radians. Adjust these values to create different conducting styles (e.g., more dramatic gestures, different arm heights, etc.).

## Technical Details

### Architecture
- **Class-based design**: `G1Conductor` class encapsulates all functionality
- **Threaded control**: Separate thread for real-time control at 50 Hz
- **State management**: Tracks robot state via DDS subscriber
- **CRC validation**: All commands include checksum validation

### Beat Calculation
The script calculates the current beat position based on elapsed time:
1. Determines which beat (1-4) is active
2. Calculates progress within that beat (0.0-1.0)
3. Applies cubic easing function
4. Interpolates between current and next beat positions

## License

This project uses the Unitree SDK2, which is subject to Unitree's licensing terms.

## Credits

Developed for Unitree G1 humanoid robot using the official Unitree SDK2 Python package.

## Support

For issues with:
- This script: Create an issue in this repository
- The G1 robot: Contact Unitree support at https://support.unitree.com
- The Unitree SDK: Visit https://github.com/unitreerobotics/unitree_sdk2_python