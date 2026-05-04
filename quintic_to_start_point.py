# # import pandas as pd
# # from franky import Robot, CartesianMotion, Affine

# # # The name of the CSV you generated from your project
# # CSV_FILENAME = "exp3b_trajectory.csv" 
# # ROBOT_IP = "192.168.1.12" # Make sure this matches your active IP

# # def main():
# #     print(f"1. Reading Line 1 of {CSV_FILENAME}...")
# #     try:
# #         df = pd.read_csv(CSV_FILENAME)
# #         first_row = df.iloc[0]
# #     except Exception as e:
# #         print(f"❌ Failed to load CSV: {e}")
# #         return

# #     # Extract coordinates from the first row
# #     target_x, target_y, target_z = first_row['x'], first_row['y'], first_row['z']
# #     qw, qx, qy, qz = first_row['qw'], first_row['qx'], first_row['qy'], first_row['qz']
    
# #     print(f"   Target Start Pose: X={target_x:.3f}, Y={target_y:.3f}, Z={target_z:.3f}")
    
# #     print("\n2. Connecting to real Franka...")
# #     robot = Robot(ROBOT_IP)
# #     robot.recover_from_errors()
    
# #     print("3. Moving robot to the exact starting line (Please stand back)...")
    
# #     # ⚠️ IMPORTANT: franky's Affine expects quaternion in [qx, qy, qz, qw] order!
# #     target_affine = Affine([target_x, target_y, target_z], [qx, qy, qz, qw])
# #     motion = CartesianMotion(target_affine)
    
# #     # Run the internal Quintic planner very slowly (5% speed) for safety
# #     robot.relative_dynamics_factor = 0.05 
    
# #     try:
# #         robot.move(motion)
# #         print("\n✅ Robot is now perfectly positioned at Line 1!")
# #         print("👉 You may now run your C++ script.")
# #     except Exception as e:
# #         print(f"\n❌ Movement blocked: {e}")

# # if __name__ == "__main__":
# #     main()





# # import pandas as pd
# # import numpy as np
# # import roboticstoolbox as rtb
# # from spatialmath import SE3, UnitQuaternion
# # from franky import Robot, JointMotion

# # CSV_FILENAME = "exp3b_trajectory.csv" # Your trajectory file
# # ROBOT_IP = "192.168.1.12" # Check your IP

# # # The "Seed" - This guarantees the solver picks an "Elbow Up" posture
# # SAFE_SEED_JOINTS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

# # def main():
# #     print(f"1. Reading Line 1 of {CSV_FILENAME}...")
# #     try:
# #         df = pd.read_csv(CSV_FILENAME)
# #         first_row = df.iloc[0]
# #     except Exception as e:
# #         print(f"❌ Failed to load CSV: {e}")
# #         return

# #     # Extract coordinates
# #     target_x, target_y, target_z = first_row['x'], first_row['y'], first_row['z']
# #     qw, qx, qy, qz = first_row['qw'], first_row['qx'], first_row['qy'], first_row['qz']
    
# #     print(f"   Target: X={target_x:.3f}, Y={target_y:.3f}, Z={target_z:.3f}")

# #     # ---------------------------------------------------------
# #     # 🧠 THE SAFE IK SOLVER
# #     # ---------------------------------------------------------
# #     print("\n2. Calculating Safe 'Elbow Up' Joint Angles...")
# #     panda = rtb.models.DH.Panda()
    
# #     # UnitQuaternion takes (w, [x, y, z])
# #     T_goal = SE3(target_x, target_y, target_z) * UnitQuaternion(qw, [qx, qy, qz]).SE3()
    
# #     sol = panda.ikine_LM(T_goal, q0=SAFE_SEED_JOINTS)

# #     if not sol.success:
# #         print("❌ IK Failed: The starting coordinate is physically unreachable.")
# #         return

# #     joint_angles = sol.q
# #     print("   ✅ Math solved! Joint angles found.")

# #     # ---------------------------------------------------------
# #     # 🛡️ VIRTUAL SAFETY CHECK (Anti-Table)
# #     # ---------------------------------------------------------
# #     print("3. Running Virtual Collision Check...")
# #     link_poses = panda.fkine_all(joint_angles)
    
# #     is_safe = True
# #     for i, pose in enumerate(link_poses):
# #         link_z_height = pose.t[2] 
# #         # Only check Links 3+ (Elbow/Wrist/Hand) to ensure they stay above 5cm
# #         if i > 2 and link_z_height < 0.05:
# #             print(f"   ⚠️ DANGER: Link {i} drops too low! (Z = {link_z_height:.3f}m)")
# #             is_safe = False

# #     if not is_safe:
# #         print("❌ ABORTING: The target posture forces the arm into the table.")
# #         return
        
# #     print("   ✅ Virtual check passed! No table collisions predicted.")

# #     # ---------------------------------------------------------
# #     # 🚀 HARDWARE DEPLOYMENT
# #     # ---------------------------------------------------------
# #     print("\n4. Connecting to real Franka...")
# #     try:
# #         robot = Robot(ROBOT_IP)
# #         robot.recover_from_errors()
        
# #         print("5. Arcing joints to start line (Please stand back)...")
# #         # Super slow dynamics for safety
# #         robot.relative_dynamics_factor = 0.05 
        
# #         # We use JointMotion so the robot naturally arcs its elbow, avoiding dips
# #         robot.move(JointMotion(joint_angles))
        
# #         print("\n✅ Robot is safely positioned at Line 1!")
# #         print("👉 You may now run your C++ script.")
        
# #     except Exception as e:
# #         print(f"\n❌ Hardware Error: {e}")

# # if __name__ == "__main__":
# #     main()

# ############################

# # import pandas as pd
# # from franky import Robot, CartesianMotion, Affine

# # CSV_FILENAME = "exp3b_trajectory.csv"
# # ROBOT_IP = "192.168.1.12"

# # def main():
# #     print(f"1. Reading Line 1 of {CSV_FILENAME}...")
# #     df = pd.read_csv(CSV_FILENAME)
# #     first_row = df.iloc[0]

# #     # Target values from your CSV
# #     t_x, t_y, t_z = first_row['x'], first_row['y'], first_row['z']
    
# #     # ⚠️ franky strictly requires [qx, qy, qz, qw] for its Affine function
# #     t_q = [first_row['qx'], first_row['qy'], first_row['qz'], first_row['qw']]

# #     print("\n2. Connecting to real Franka...")
# #     robot = Robot(ROBOT_IP)
# #     robot.recover_from_errors()
# #     robot.relative_dynamics_factor = 0.05 # 5% speed for absolute safety

# #     # Get current position
# #     curr_pose = robot.current_cartesian_state.pose.end_effector_pose
# #     c_x, c_y, c_z = curr_pose.translation
# #     c_q = curr_pose.quaternion

# #     # ---------------------------------------------------------
# #     # 🏗️ THE 3-STEP CRANE MANEUVER
# #     # ---------------------------------------------------------
    
# #     # STEP 1: LIFT STRAIGHT UP (Keep current orientation)
# #     print("3. Phase 1: Lifting arm straight up to safe altitude (Z=0.55m)...")
# #     safe_z = max(c_z, 0.55) # Go to 55cm high
# #     lift_motion = CartesianMotion(Affine([c_x, c_y, safe_z], c_q))
# #     robot.move(lift_motion)

# #     # STEP 2: ROTATE AND MOVE X/Y IN THE AIR
# #     print("4. Phase 2: Rotating wrist to upright pose while in the air...")
# #     hover_motion = CartesianMotion(Affine([t_x, t_y, safe_z], t_q))
# #     robot.move(hover_motion)

# #     # STEP 3: LOWER GENTLY TO TARGET
# #     print(f"5. Phase 3: Lowering gently to starting Z={t_z:.3f}m...")
# #     final_motion = CartesianMotion(Affine([t_x, t_y, t_z], t_q))
# #     robot.move(final_motion)

# #     print("\n✅ Robot is safely positioned at Line 1!")
# #     print("👉 You may now run your C++ script.")

# # if __name__ == "__main__":
# #     main()


# # from franky import Robot, CartesianMotion, Affine

# # # Make sure this matches your active IP
# # ROBOT_IP = "192.168.1.12" 

# # # =========================================================
# # # 🎯 HARDCODED TARGET POSTURE
# # # =========================================================
# # # Currently set to your manual "sideways/holding" test coordinates.
# # TARGET_X = 0.47181
# # TARGET_Y = -0.30506
# # TARGET_Z = 0.23034

# # # ⚠️ franky strictly requires [qx, qy, qz, qw] format!
# # TARGET_Q = [-0.50892, -0.49164, -0.48914, 0.50994]
# # # =========================================================

# # def main():
# #     print("1. Target explicitly set in script (No CSV).")
# #     t_x, t_y, t_z = TARGET_X, TARGET_Y, TARGET_Z
# #     t_q = TARGET_Q
    
# #     print(f"   Target: X={t_x:.3f}, Y={t_y:.3f}, Z={t_z:.3f}")

# #     print("\n2. Connecting to real Franka...")
# #     try:
# #         robot = Robot(ROBOT_IP)
# #         robot.recover_from_errors()
# #         robot.relative_dynamics_factor = 0.05 # 5% speed for absolute safety
# #     except Exception as e:
# #         print(f"❌ Connection error: {e}")
# #         return

# #     # Get current physical position
# #     curr_pose = robot.current_cartesian_state.pose.end_effector_pose
# #     c_x, c_y, c_z = curr_pose.translation
# #     c_q = curr_pose.quaternion

# #     # ---------------------------------------------------------
# #     # 🏗️ THE 3-STEP CRANE MANEUVER
# #     # ---------------------------------------------------------
    
# #     # STEP 1: LIFT STRAIGHT UP (Keep current orientation)
# #     print("3. Phase 1: Lifting arm straight up to safe altitude (Z=0.55m)...")
# #     safe_z = max(c_z, 0.55) # Go to 55cm high
# #     try:
# #         lift_motion = CartesianMotion(Affine([c_x, c_y, safe_z], c_q))
# #         robot.move(lift_motion)
# #     except Exception as e:
# #         print(f"❌ Phase 1 failed: {e}")
# #         return

# #     # STEP 2: ROTATE AND MOVE X/Y IN THE AIR
# #     print("4. Phase 2: Rotating wrist to target pose while in the air...")
# #     try:
# #         hover_motion = CartesianMotion(Affine([t_x, t_y, safe_z], t_q))
# #         robot.move(hover_motion)
# #     except Exception as e:
# #         print(f"❌ Phase 2 failed: {e}")
# #         return

# #     # STEP 3: LOWER GENTLY TO TARGET
# #     print(f"5. Phase 3: Lowering gently to starting Z={t_z:.3f}m...")
# #     try:
# #         final_motion = CartesianMotion(Affine([t_x, t_y, t_z], t_q))
# #         robot.move(final_motion)
# #     except Exception as e:
# #         print(f"❌ Phase 3 failed: {e}")
# #         return

# #     print("\n✅ Robot is safely positioned at the hardcoded target!")

# # if __name__ == "__main__":
# #     main()


# import pandas as pd
# import numpy as np
# import roboticstoolbox as rtb
# from spatialmath import SE3, UnitQuaternion
# from franky import Robot, JointMotion

# # The script will dynamically read whatever CSV you put here
# CSV_FILENAME = "exp3b_trajectory.csv" 
# ROBOT_IP = "192.168.1.12" 

# # The "Seed" - This mathematically forces the IK solver to pick an "Elbow Up" posture
# SAFE_SEED_JOINTS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

# def main():
#     print(f"1. Reading Line 1 of {CSV_FILENAME}...")
#     try:
#         df = pd.read_csv(CSV_FILENAME)
#         first_row = df.iloc[0]
#     except Exception as e:
#         print(f"❌ Failed to load CSV: {e}")
#         return

#     # Extract coordinates directly from your trajectory
#     target_x, target_y, target_z = first_row['x'], first_row['y'], first_row['z']
#     qw, qx, qy, qz = first_row['qw'], first_row['qx'], first_row['qy'], first_row['qz']
    
#     print(f"   Target: X={target_x:.3f}, Y={target_y:.3f}, Z={target_z:.3f}")

#     # ---------------------------------------------------------
#     # 🧠 THE SAFE IK SOLVER (Finds the optimal joint angles)
#     # ---------------------------------------------------------
#     print("\n2. Calculating Safe 'Elbow Up' Joint Angles...")
#     panda = rtb.models.DH.Panda()
    
#     # spatialmath UnitQuaternion strictly takes (w, [x, y, z])
#     T_goal = SE3(target_x, target_y, target_z) * UnitQuaternion(qw, [qx, qy, qz]).SE3()
    
#     sol = panda.ikine_LM(T_goal, q0=SAFE_SEED_JOINTS)

#     if not sol.success:
#         print("❌ IK Failed: The starting coordinate is physically unreachable.")
#         return

#     joint_angles = sol.q
#     print("   ✅ Math solved! Joint angles found.")

#     # ---------------------------------------------------------
#     # 🛡️ VIRTUAL SAFETY CHECK (Mathematical Anti-Table Shield)
#     # ---------------------------------------------------------
#     print("3. Running Virtual Collision Check...")
#     link_poses = panda.fkine_all(joint_angles)
    
#     is_safe = True
#     for i, pose in enumerate(link_poses):
#         link_z_height = pose.t[2] 
#         # Only check Links 3+ (Elbow/Wrist/Hand) to ensure they stay strictly above 5cm
#         if i > 2 and link_z_height < 0.05:
#             print(f"   ⚠️ DANGER: Link {i} drops too low! (Z = {link_z_height:.3f}m)")
#             is_safe = False

#     if not is_safe:
#         print("❌ ABORTING: The target posture forces the arm into the table.")
#         return
        
#     print("   ✅ Virtual check passed! No table collisions predicted.")

#     # ---------------------------------------------------------
#     # 🚀 HARDWARE DEPLOYMENT
#     # ---------------------------------------------------------
#     print("\n4. Connecting to real Franka...")
#     try:
#         robot = Robot(ROBOT_IP)
#         robot.recover_from_errors()
        
#         print("5. Arcing joints to start line (Please stand back)...")
#         # Super slow dynamics for maximum safety during setup
#         robot.relative_dynamics_factor = 0.05 
        
#         # JointMotion safely arcs all 7 joints simultaneously to the solved configuration
#         robot.move(JointMotion(joint_angles))
        
#         print("\n✅ Robot is safely positioned at Line 1!")
#         print("👉 You may now run your C++ script.")
        
#     except Exception as e:
#         print(f"\n❌ Hardware Error: {e}")

# if __name__ == "__main__":
#     main()

import pandas as pd
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion
from franky import Robot, JointMotion

# Will dynamically adapt to ANY trajectory you generate in the future
CSV_FILENAME = "exp2_trajectory.csv" 
ROBOT_IP = "192.168.1.12" 

def main():
    print(f"1. Reading Line 1 of {CSV_FILENAME}...")
    try:
        df = pd.read_csv(CSV_FILENAME)
        first_row = df.iloc[0]
    except Exception as e:
        print(f"❌ Failed to load CSV: {e}")
        return

    # Extract coordinates
    t_x, t_y, t_z = first_row['x'], first_row['y'], first_row['z']
    qw, qx, qy, qz = first_row['qw'], first_row['qx'], first_row['qy'], first_row['qz']
    
    print(f"   Target: X={t_x:.3f}, Y={t_y:.3f}, Z={t_z:.3f}")

    # ---------------------------------------------------------
    # 🧠 THE MULTI-START "NULL SPACE" OPTIMIZER
    # ---------------------------------------------------------
    print("\n2. Scanning for the absolute best 'Elbow-Up' configuration...")
    panda = rtb.models.DH.Panda()
    T_goal = SE3(t_x, t_y, t_z) * UnitQuaternion(qw, [qx, qy, qz]).SE3()
    
    successful_solutions = []

    # Generate 50 randomized starting seeds to force the math to explore different bends
    np.random.seed(42) # Keep it reproducible
    for _ in range(50):
        # Random joints within Franka's physical limits
        random_seed = np.random.uniform(panda.qlim[0], panda.qlim[1])
        
        # We use ikine_LM (Levenberg-Marquardt) for robust numerical solving
        sol = panda.ikine_LM(T_goal, q0=random_seed)
        
        if sol.success:
            # If it reached the target, calculate where the elbow is physically located
            link_poses = panda.fkine_all(sol.q)
            elbow_z = link_poses[4].t[2] # Link 4 is the Elbow
            
            # Save the joint angles and the elbow height
            successful_solutions.append({
                'joints': sol.q,
                'elbow_z': elbow_z
            })

    if not successful_solutions:
        print("\n❌ ABORTING: Scanned 50 configurations. This target is physically unreachable.")
        return

    # 🏆 Select the configuration with the HIGHEST elbow
    best_solution = max(successful_solutions, key=lambda x: x['elbow_z'])
    best_joints = best_solution['joints']
    max_elbow_height = best_solution['elbow_z']
    
    print(f"   ✅ Found {len(successful_solutions)} valid ways to reach the target.")
    print(f"   ✅ Selected the optimal posture (Elbow Height: {max_elbow_height:.3f}m).")

    # ---------------------------------------------------------
    # 🚀 SMOOTH QUINTIC HARDWARE DEPLOYMENT
    # ---------------------------------------------------------
    print("\n3. Connecting to real Franka...")
    try:
        robot = Robot(ROBOT_IP)
        robot.recover_from_errors()
        
        print("4. Arcing joints safely to start line (Please stand back)...")
        robot.relative_dynamics_factor = 0.10 # 10% speed for setup
        
        # JointMotion automatically uses Franka's internal smooth planner
        robot.move(JointMotion(best_joints))
        
        print("\n✅ Robot is perfectly positioned with the elbow high!")
        print("👉 You may now safely run your C++ impedance controller.")
        
    except Exception as e:
        print(f"\n❌ Hardware Error: {e}")

if __name__ == "__main__":
    main()