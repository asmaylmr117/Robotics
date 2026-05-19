"""
=============================================================
  PyBullet Robot Simulation – Square Path
  
  Project Requirements:
    - Move robot along a square path
    - Print position (x, y, z) and orientation at each step
    - Compare start vs end position
    - Bonus: Calculate distance between start and end
=============================================================
"""

import pybullet as p
import pybullet_data
import time
import math


# ─────────────────────────────────────────────
#  HELPER FUNCTIONS
# ─────────────────────────────────────────────

def print_robot_state(robot, step_label=""):
    """Read and print the robot's current position and orientation."""
    position, orientation = p.getBasePositionAndOrientation(robot)
    print(f"  [{step_label}]")
    print(f"    Position:    ({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f})")
    print(f"    Orientation: ({orientation[0]:.4f}, {orientation[1]:.4f}, "
          f"{orientation[2]:.4f}, {orientation[3]:.4f})")
    return position, orientation


def get_robot_heading(robot):
    """Get the current yaw (heading) of the robot in degrees."""
    _, orientation = p.getBasePositionAndOrientation(robot)
    euler = p.getEulerFromQuaternion(orientation)
    return math.degrees(euler[2])  # Yaw is the 3rd element


def set_wheel_velocity(robot, left_speed, right_speed):
    """
    Apply velocity to the robot's wheels.
    Assumes a differential-drive robot (e.g., R2D2 / husky) with
    joint indices 2 (left) and 3 (right). Adjust if needed.
    """
    num_joints = p.getNumJoints(robot)
    
    # Collect wheel joints by name
    left_joints = []
    right_joints = []
    for i in range(num_joints):
        info = p.getJointInfo(robot, i)
        name = info[1].decode('utf-8').lower()
        if "wheel" in name:
            if "left" in name:
                left_joints.append(i)
            elif "right" in name:
                right_joints.append(i)

    # If no "wheel" joints found, use the fallback splitting logic
    if not left_joints and not right_joints:
        wheel_joints = []
        for i in range(num_joints):
            info = p.getJointInfo(robot, i)
            if info[2] == p.JOINT_REVOLUTE:
                wheel_joints.append(i)
        half = len(wheel_joints) // 2
        left_joints  = wheel_joints[:half] if half > 0 else wheel_joints
        right_joints = wheel_joints[half:] if half > 0 else wheel_joints

    for j in left_joints:
        p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL,
                                targetVelocity=left_speed, force=1000)
    for j in right_joints:
        p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL,
                                targetVelocity=right_speed, force=1000)


def move_distance(robot, distance, speed=8.0, time_step=1./240.):
    """Drive the robot forward with P-control for accuracy."""
    start_pos, _ = p.getBasePositionAndOrientation(robot)
    traveled = 0
    print(f"\n  [Moving forward {distance}m...]")
    
    while traveled < distance:
        # P-control: slow down when close to target
        remaining = distance - traveled
        current_speed = speed
        if remaining < 0.5:
            current_speed = max(1.0, speed * (remaining / 0.5))
            
        set_wheel_velocity(robot, current_speed, current_speed)
        p.stepSimulation()
        # time.sleep(time_step)
        
        current_pos, _ = p.getBasePositionAndOrientation(robot)
        traveled = math.sqrt((current_pos[0] - start_pos[0])**2 + 
                            (current_pos[1] - start_pos[1])**2)
        
        if int(traveled * 100) > 0 and int(traveled * 100) % 50 == 0:
             print_robot_state(robot, step_label=f"moving dist={traveled:.2f}m")

    set_wheel_velocity(robot, 0, 0)


def turn_angle(robot, target_angle_degrees, turn_speed=5.0, time_step=1./240.):
    """Turn the robot with P-control for accuracy."""
    start_heading = get_robot_heading(robot)
    rotated = 0
    print(f"\n  [Turning left {target_angle_degrees} deg...]")

    while abs(rotated) < target_angle_degrees:
        # P-control: slow down when close to target angle
        remaining = target_angle_degrees - abs(rotated)
        current_turn_speed = turn_speed
        if remaining < 20:
            current_turn_speed = max(0.5, turn_speed * (remaining / 20))

        set_wheel_velocity(robot, -current_turn_speed, current_turn_speed)
        p.stepSimulation()
        # time.sleep(time_step)
        
        current_heading = get_robot_heading(robot)
        diff = current_heading - start_heading
        while diff > 180: diff -= 360
        while diff < -180: diff += 360
        rotated = diff
        
    set_wheel_velocity(robot, 0, 0)


def euclidean_distance(pos1, pos2):
    """Bonus: 2-D Euclidean distance between two positions."""
    dx = pos2[0] - pos1[0]
    dy = pos2[1] - pos1[1]
    return math.sqrt(dx**2 + dy**2)


# ─────────────────────────────────────────────
#  SIMULATION SETUP
# ─────────────────────────────────────────────

def setup_simulation():
    """Initialise PyBullet, load the plane and a robot."""
    # Connect to PyBullet with a GUI window
    physics_client = p.connect(p.GUI)

    # Point PyBullet to its built-in assets (plane, robots, etc.)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Set gravity (Earth: -9.81 m/s² on Z axis)
    p.setGravity(0, 0, -9.81)

    # Load a flat ground plane
    plane_id = p.loadURDF("plane.urdf")

    # Load the R2D2 robot (comes with pybullet_data; easy to work with)
    start_pos        = [0, 0, 0.5]          # slightly higher to avoid clipping
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])  # no initial rotation
    robot = p.loadURDF("r2d2.urdf", start_pos, start_orientation)

    # Set simulation time step
    p.setTimeStep(1. / 240.)

    return robot


# ─────────────────────────────────────────────
#  MAIN PROGRAM
# ─────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  PyBullet Robot Simulation  -  Square Path")
    print("=" * 60)

    # ── 1. Setup ────────────────────────────────────────────
    robot = setup_simulation()

    # ── 2. Record starting state ────────────────────────────
    print("\n>>> STARTING STATE")
    start_pos, start_orient = print_robot_state(robot, step_label="START")

    # Let the simulation settle for a moment
    for _ in range(120):
        p.stepSimulation()
        # time.sleep(1. / 240.)

    # ── 3. Execute square path (4 sides × 90° turns) ────────
    print("\n>>> EXECUTING SQUARE PATH")
    print("    Each side: move forward -> turn left 90 deg\n")

    for side in range(1, 5):
        print(f"\n  -- Side {side} of 4 --")
        move_distance(robot, distance=2.0, speed=8.0)
        
        # Small pause for stability
        for _ in range(10): p.stepSimulation(); time.sleep(1./240.)
        
        print(f"\n  -- Turn {side} (left 90 deg) --")
        turn_angle(robot, target_angle_degrees=90, turn_speed=5.0)

        # Small pause for stability
        for _ in range(10): p.stepSimulation(); time.sleep(1./240.)

    # ── 4. Stop the robot ───────────────────────────────────
    set_wheel_velocity(robot, 0, 0)
    for _ in range(60):      # let it fully stop
        p.stepSimulation()
        time.sleep(1. / 240.)

    # ── 5. Record ending state ──────────────────────────────
    print("\n>>> ENDING STATE")
    end_pos, end_orient = print_robot_state(robot, step_label="END")

    # ── 6. Compare start vs end ─────────────────────────────
    print("\n>>> COMPARISON: START vs END")
    print(f"    Start Position : ({start_pos[0]:.4f}, {start_pos[1]:.4f}, {start_pos[2]:.4f})")
    print(f"    End   Position : ({end_pos[0]:.4f},   {end_pos[1]:.4f},   {end_pos[2]:.4f})")

    threshold = 0.5   # metres – tolerance for "returned to start"
    dist = euclidean_distance(start_pos, end_pos)

    print(f"\n    Euclidean distance (2-D): {dist:.4f} m")

    if dist < threshold:
        print("    [OK]  Robot RETURNED to the starting position (within threshold).")
    else:
        print("    [FAIL]  Robot did NOT fully return to start.")
        print(f"        Remaining offset: {dist:.4f} m")
        print("        Reason: Wheel slip, simulation friction, and imprecise")
        print("        turn duration accumulate errors over 4 sides + 4 turns.")

    # ── 7. Keep window open so user can see the robot ───────
    print("\n>>> Simulation complete. Close the PyBullet window to exit.")
    try:
        while p.isConnected():
            p.stepSimulation()
            time.sleep(1. / 60.)
    except Exception:
        pass

    if p.isConnected():
        p.disconnect()
    print("Disconnected. Goodbye!")


if __name__ == "__main__":
    main()
