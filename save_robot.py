from franky import Robot, JointMotion

# The standard, safe "folded" position right in the middle of the workspace
SAFE_HOME_JOINTS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

def main():
    print("Connecting to Franka to release singularity lock...")
    try:
        robot = Robot("192.168.1.12")
        robot.recover_from_errors()
        
        print("Pulling arm back to center (10 speed)...")
        robot.relative_dynamics_factor = 0.10 
        
        # Power through the brakes using Execution Mode
        robot.move(JointMotion(SAFE_HOME_JOINTS))
        
        print("\n✅ Robot is successfully rescued and back in the safe zone!")
        print("Guiding mode will now work normally.")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")

if __name__ == "__main__":
    main()