from franky import Robot

def main():
    print("Connecting to Franka...")
    try:
        # Using the IP address from your previous scripts
        robot = Robot("192.168.1.12")
        
        # Grab the current physical state
        pose = robot.current_cartesian_state.pose.end_effector_pose
        
        # Extract the exact values
        x, y, z = pose.translation
        
        # Remember: franky outputs in [x, y, z, w] order
        qx, qy, qz, qw = pose.quaternion 
        
        print("\n--- CURRENT END EFFECTOR POSE ---")
        print(f"Position (meters):")
        print(f"  X: {x:.5f}")
        print(f"  Y: {y:.5f}")
        print(f"  Z: {z:.5f}")
        
        print(f"\nOrientation (Quaternions):")
        print(f"  qw: {qw:.5f} (W is printed first for your C++ CSV format)")
        print(f"  qx: {qx:.5f}")
        print(f"  qy: {qy:.5f}")
        print(f"  qz: {qz:.5f}")
        print("---------------------------------\n")

    except Exception as e:
        print(f"Connection error: {e}. Is the robot powered on and unlocked?")

if __name__ == "__main__":
    main()
