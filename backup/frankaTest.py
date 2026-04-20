import sys
import time
from franky import Robot, Affine, CartesianMotion, ReferenceType

# --- CONFIGURATION ---
ROBOT_IP = "192.168.1.12"  
MOVE_DISTANCE = 0.05     # 5 cm
SPEED_SCALE = 0.05       # 5% Speed 

def main():
    print(f"1. Connecting to robot at {ROBOT_IP}...")
    try:
        # REMOVED RealtimeConfig.Ignore! Your system is full RT now.
        robot = Robot(ROBOT_IP)
        print("    Connected successfully.")
    except Exception as e:
        print(f"    Connection failed: {e}")
        return

    print("2. Checking robot status...")
    try:
        robot.recover_from_errors()
        print("    Robot recovered.")
    except Exception as e:
        print(f"   Recovery failed: {e}")
        return

    try:
        current_state = robot.current_cartesian_state
        print(f"    Current Pose:\n{current_state.pose}") 
    except Exception as e:
        print(f"    Could not read pose: {e}")

    robot.relative_dynamics_factor = SPEED_SCALE
    print(f"    Dynamics limited to {int(SPEED_SCALE*100)}%")

    # Changed to X-axis for safety (sliding motion)
    target_transform = Affine([MOVE_DISTANCE, 0.0, 0.0])
    motion_forward = CartesianMotion(target_transform, ReferenceType.Relative)

    print(f"\n  WARNING: Robot is about to move {int(MOVE_DISTANCE*100)}cm along its X-axis.")
    print("   -> Keep your hand on the E-Stop.")
    input("   -> Press [ENTER] to execute movement...")

    print("7. Moving...")
    try:
        robot.move(motion_forward)
        print("    Move Complete.")
    except Exception as e:
        print(f"    Movement failed: {e}")
        return

    print("\n   -> Would you like to go back to the start? (y/n)")
    choice = input("   -> ")
    if choice.lower() == 'y':
        print("   Moving BACK...")
        motion_backward = CartesianMotion(target_transform.inverse, ReferenceType.Relative)
        robot.move(motion_backward)
        print("   Back at start.")
    
    print("Test Complete.")

if __name__ == "__main__":
    main()
