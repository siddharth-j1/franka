# from franky import Robot, CartesianMotion, Affine

# # 1. Connect and Clear Errors
# robot = Robot("192.168.1.15")
# robot.recover_from_errors() 

# # 2. Safety First: Set speed to very slow (10%)
# # This prevents any sudden jerks while testing new positions
# robot.relative_dynamics_factor = 0.1 

# # 3. Define your target coordinates
# # Using your exact recorded X, Y, Z and Quaternions (x, y, z, w format)
# target_pose = Affine(
#     translation=[0.36845, -0.07430, 0.12793],
#     quaternion=[0.50683, 0.50370, 0.49214, -0.49720]
# )

# print("Moving robot to recorded coordinates...")
# try:
#     # We use CartesianMotion here instead of JointMotion
#     robot.move(CartesianMotion(target_pose))
#     print("✅ Robot reached the target position successfully.")
# except Exception as e:
#     print(f"❌ Move failed: {e}")


# above is working fine fromhome position 

#test code below
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion
from franky import Robot, JointMotion

# =========================================================
# 🎯 TARGET COORDINATES
# =========================================================
TARGET_X, TARGET_Y, TARGET_Z = 0.36845, -0.07430, 0.12793
TARGET_ORI = UnitQuaternion(-0.49720, [0.50683, 0.50370, 0.49214])

# The "Seed" - This guarantees the solver picks an "Elbow Up" posture
SAFE_SEED_JOINTS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
# =========================================================

def main():
    panda = rtb.models.DH.Panda()
    T_goal = SE3(TARGET_X, TARGET_Y, TARGET_Z) * TARGET_ORI.SE3()

    print("1. Solving IK with 'Elbow Up' Safe Seed...")
    
    # We use ikine_LM (Levenberg-Marquardt) and provide the safe starting guess (q0)
    sol = panda.ikine_LM(T_goal, q0=SAFE_SEED_JOINTS)

    if not sol.success:
        print("❌ IK Failed: The target is mathematically unreachable.")
        return

    joint_angles = sol.q
    print("✅ Math solved! Joint angles found.")

    # ---------------------------------------------------------
    # 🛡️ VIRTUAL SAFETY CHECK (The "Anti-Table" System)
    # ---------------------------------------------------------
    print("\n2. Running Virtual Collision Check...")
    link_poses = panda.fkine_all(joint_angles)
    
    is_safe = True
    for i, pose in enumerate(link_poses):
        link_z_height = pose.t[2] 
        
        # SKIP the base and shoulder joints (Links 0, 1, and 2)
        # Only check if the arm (Link 3+) dips below 5cm!
        if i > 2 and link_z_height < 0.05:
            print(f"   ⚠️ DANGER: Link {i} drops too low! (Z = {link_z_height:.3f}m)")
            is_safe = False

    if not is_safe:
        print("❌ ABORTING MOVE: The math predicts the arm will hit the table.")
        return
        
    print("   ✅ Virtual check passed! No table collisions predicted.")
    # ---------------------------------------------------------
    # 3. DEPLOY TO HARDWARE
    # ---------------------------------------------------------
    print("\n3. Connecting to real Franka...")
    try:
        robot = Robot("192.168.1.15")
        robot.recover_from_errors()
        
        # Super slow dynamics for testing new IK points
        robot.relative_dynamics_factor = 0.05 
        
        input("👉 Press [ENTER] to execute safe JointMotion...")
        
        robot.move(JointMotion(joint_angles))
        print("✅ Target reached safely!")
        
    except Exception as e:
        print(f"❌ Hardware Error: {e}")

if __name__ == "__main__":
    main()