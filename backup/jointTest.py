import sys
import time
import signal
import faulthandler

# 1. Enable Crash Reporting (This will print WHERE it crashed if it happens again)
faulthandler.enable()

try:
    from franky import Robot, JointMotion, RealtimeConfig
except ImportError:
    print(" Error: Library not found.")
    sys.exit(1)

ROBOT_IP = "192.168.1.15"

def main():
    print(f"1. Connecting to {ROBOT_IP}...")
    # NOTE: We use RealtimeConfig.Ignore as before
    robot = Robot(ROBOT_IP, realtime_config=RealtimeConfig.Ignore)
    
    # 2. Recover & Read State
    robot.recover_from_errors()
    
    # Read current joint angles (7 numbers)
    q_current = robot.current_joint_positions
    print(f"   Current Joints: {q_current}")
    
    # 3. Create Target (Move Joint 7 by +0.1 radians ~ 5 degrees)
    q_target = list(q_current)
    q_target[6] += 0.1 
    
    # 4. Limit Speed severely
    robot.relative_dynamics_factor = 0.05
    
    print("\n  WARNING: Robot will rotate WRIST (Joint 7) slightly.")
    input("   -> Press [ENTER] to execute...")

    # 5. Move using JOINT motion (Simpler than Cartesian)
    print("5. Moving Joints...")
    try:
        motion = JointMotion(q_target)
        robot.move(motion)
        print("   Joint Move Complete!")
    except Exception as e:
        print(f"   Error: {e}")

    # 6. Move Back
    print("6. Moving Back...")
    motion_back = JointMotion(list(q_current))
    robot.move(motion_back)
    print("   Done.")

if __name__ == "__main__":
    main()