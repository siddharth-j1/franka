
from franky import Robot, CartesianMotion, Affine

# Make sure this matches your active IP
ROBOT_IP = "192.168.1.12" 

# =========================================================
# 🎯 HARDCODED TARGET POSTURE
# =========================================================
# Currently set to your manual "sideways/holding" test coordinates.
TARGET_X = 0.36845000
TARGET_Y = -0.07430000
TARGET_Z = 0.12793000

# ⚠️ franky strictly requires [qx, qy, qz, qw] format!
TARGET_Q = [-0.50892, -0.49164, -0.48914, 0.50994]
# =========================================================

def main():
    print("1. Target explicitly set in script (No CSV).")
    t_x, t_y, t_z = TARGET_X, TARGET_Y, TARGET_Z
    t_q = TARGET_Q
    
    print(f"   Target: X={t_x:.3f}, Y={t_y:.3f}, Z={t_z:.3f}")

    print("\n2. Connecting to real Franka...")
    try:
        robot = Robot(ROBOT_IP)
        robot.recover_from_errors()
        robot.relative_dynamics_factor = 0.05 # 5% speed for absolute safety
    except Exception as e:
        print(f"❌ Connection error: {e}")
        return

    # Get current physical position
    curr_pose = robot.current_cartesian_state.pose.end_effector_pose
    c_x, c_y, c_z = curr_pose.translation
    c_q = curr_pose.quaternion

    # ---------------------------------------------------------
    # 🏗️ THE 3-STEP CRANE MANEUVER
    # ---------------------------------------------------------
    
    # STEP 1: LIFT STRAIGHT UP (Keep current orientation)
    print("3. Phase 1: Lifting arm straight up to safe altitude (Z=0.55m)...")
    safe_z = max(c_z, 0.55) # Go to 55cm high
    try:
        lift_motion = CartesianMotion(Affine([c_x, c_y, safe_z], c_q))
        robot.move(lift_motion)
    except Exception as e:
        print(f"❌ Phase 1 failed: {e}")
        return

    # STEP 2: ROTATE AND MOVE X/Y IN THE AIR
    print("4. Phase 2: Rotating wrist to target pose while in the air...")
    try:
        hover_motion = CartesianMotion(Affine([t_x, t_y, safe_z], t_q))
        robot.move(hover_motion)
    except Exception as e:
        print(f"❌ Phase 2 failed: {e}")
        return

    # STEP 3: LOWER GENTLY TO TARGET
    print(f"5. Phase 3: Lowering gently to starting Z={t_z:.3f}m...")
    try:
        final_motion = CartesianMotion(Affine([t_x, t_y, t_z], t_q))
        robot.move(final_motion)
    except Exception as e:
        print(f"❌ Phase 3 failed: {e}")
        return

    print("\n✅ Robot is safely positioned at the hardcoded target!")

if __name__ == "__main__":
    main()