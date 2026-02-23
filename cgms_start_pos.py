from franky import Robot, JointMotion, Affine, CartesianMotion, ReferenceType

# 1. Connect and Clear Errors
robot = Robot("192.168.1.15")
robot.recover_from_errors() 

# 2. Safety First: Set speed to very slow (10%)
robot.relative_dynamics_factor = 0.1 

# Standard "Ready" Joint Configuration
ready_joints = [0.0, 0.0, 0.0, -2.2, 0.0, 2.2, 0.7]

try:
    # --- PHASE 1: Untangle using Joints ---
    print("Phase 1: Moving robot to safe default joints to untangle...")
    robot.move(JointMotion(ready_joints))
    print("Robot reset successfully.")

    # --- PHASE 2: Move to the Exact Paper Start Position ---
    print("\nPhase 2: Moving to exact C-GMS starting coordinate [0.55, 0.00, 0.11]...")
    
    # Grab the current safe orientation so the gripper doesn't twist
    current_pose = robot.current_cartesian_state.pose
    safe_orientation = current_pose.end_effector_pose.quaternion

    # Define the exact starting coordinate
    target_translation = [0.55, 0.00, 0.11]
    target_affine = Affine(translation=target_translation, quaternion=safe_orientation)
    
    # Move to the coordinate
    motion = CartesianMotion(target_affine, ReferenceType.Absolute)
    robot.move(motion)
    print(" Robot is perfectly in position. Ready for C++ C-GMS execution!")

except Exception as e:
    print(f" Reset failed: {e}")