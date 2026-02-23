import sys
import numpy as np
from franky import (
    Robot, Affine, RobotPose, CartesianWaypoint, 
    CartesianWaypointMotion, ReferenceType
)

# --- CONFIGURATION ---
ROBOT_IP = "192.168.1.15"  
HZ = 1000  

def load_trajectory_data(start_pose):
    duration = 5.0
    steps = int(duration * HZ)
    y_des = np.zeros((steps, 3))
    
    start_pos = start_pose.end_effector_pose.translation
    target_pos = start_pos + np.array([0.05, 0.0, 0.0]) # 5cm in X
    
    for t in range(steps):
        percent = t / steps
        s = 10*(percent**3) - 15*(percent**4) + 6*(percent**5)
        y_des[t] = start_pos + s * (target_pos - start_pos)
        
    return y_des

def main():
    print(f"1. Connecting to robot at {ROBOT_IP}...")
    try:
        robot = Robot(ROBOT_IP)
        robot.recover_from_errors()
        start_pose = robot.current_cartesian_state.pose
        print("    Connected successfully.")
    except Exception as e:
        print(f"    Connection failed: {e}")
        return

    # 1. Load the mock trajectory
    y_des = load_trajectory_data(start_pose)
    start_quat = start_pose.end_effector_pose.quaternion
    elbow = start_pose.elbow_state # Keep the elbow exactly where it is

    # 2. Extract Waypoints (Downsample from 1000Hz to 100Hz for Python safety)
    # 2. Extract Sparse Waypoints (Just 5 Key Frames!)
    waypoints = []
    print("    Extracting Sparse Waypoints...")
    
    # We skip index 0 because the robot is already there!
    # We grab a frame exactly every 1000 steps (1.0 seconds)
    for i in range(1000, len(y_des), 1000):
        pose = RobotPose(Affine(translation=y_des[i], quaternion=start_quat), elbow)
        waypoints.append(CartesianWaypoint(pose, ReferenceType.Absolute))

    # Ensure the absolute final target is locked in
    final_pose = RobotPose(Affine(translation=y_des[-1], quaternion=start_quat), elbow)
    waypoints.append(CartesianWaypoint(final_pose, ReferenceType.Absolute))

    print(f"\n  HARDWARE DEPLOYMENT: {len(waypoints)} Waypoints.")
    print("   -> Distance: 5cm Forward (X)")
    input("   -> Press [ENTER] to execute Waypoint Trajectory...")

    print("7. Moving Robot...")
    try:
        # 3. Create the Motion object with the list of waypoints
        motion = CartesianWaypointMotion(waypoints)
        
        # 4. Command the robot!
        robot.move(motion)
        print("    Success! Robot completed the waypoint trajectory.")
    except Exception as e:
        print(f"    Hardware error during move: {e}")

if __name__ == "__main__":
    main()