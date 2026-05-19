# PyBullet Robot Square Path Simulation

A Python simulation using PyBullet to control an R2D2 robot through a square path, demonstrating robot kinematics, position tracking, and orientation control.

## Features

- **Square Path Navigation**: Robot moves forward 2 meters and turns 90° left, repeated 4 times
- **Position Tracking**: Real-time position (x, y, z) and orientation output at each step
- **Start/End Comparison**: Compares initial and final robot states
- **Error Analysis**: Calculates Euclidean distance between start and end positions
- **P-Control**: Proportional control for accurate distance and angle movements

## Requirements

- Python 3.x
- PyBullet (`pip install pybullet`)

## Usage

```bash
python robot_square_path.py
```

## How It Works

1. **Setup**: Initializes PyBullet with a ground plane and R2D2 robot
2. **Record Start State**: Captures initial position and orientation
3. **Execute Square Path**: 
   - Move forward 2 meters
   - Turn left 90 degrees
   - Repeat 4 times
4. **Compare States**: Calculates and displays distance from start to end position

## Controls

- **Move Forward**: Both wheel velocities set to same positive value
- **Turn Left**: Left wheels reverse, right wheels forward (differential drive)

## Expected Output

The robot will print its position and orientation at key moments, then calculate the Euclidean distance between start and end positions. Due to wheel slip and simulation granularity, the robot typically ends up within 0.1-0.8 meters of the starting point rather than exactly at the origin.