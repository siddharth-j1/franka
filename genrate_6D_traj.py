import numpy as np
import pandas as pd
from franky import Robot
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

# =========================================================
#  ENTER YOUR DESTINATION HERE
# =========================================================
# Set this to wherever you want the robot to go!
# Right now, it is set to your previously recorded Home coordinates.
TARGET_X = 0.36845
TARGET_Y = -0.07430
TARGET_Z = 0.12793

# Target Orientation in SciPy format: [qx, qy, qz, qw]
TARGET_QUAT = [0.50683, 0.50370, 0.49214, -0.49720]

# Total time for the trip (10+ seconds is best for large moves/twists)
MOVE_TIME = 12.0 
# =========================================================

def main():
    print("1. Connecting to robot to read exactly where it is now...")
    try:
        robot = Robot("192.168.1.13")
        robot.recover_from_errors() # Clear any existing red-light errors!
        pose = robot.current_cartesian_state.pose.end_effector_pose
    except Exception as e:
        print(f"❌ Connection error: {e}")
        return
        
    # Get physical starting position
    x0, y0, z0 = pose.translation
    qx0, qy0, qz0, qw0 = pose.quaternion
    start_quat_hardware = np.array([qx0, qy0, qz0, qw0])
    rot_start = R.from_quat(start_quat_hardware)
    rot_end = R.from_quat(TARGET_QUAT)
    
    print(f"2. Planning a perfectly smooth {MOVE_TIME}-second Quintic path...")
    
    hz = 1000
    steps = int(MOVE_TIME * hz)
    tau = np.linspace(0, 1, steps)
    
    # Quintic S-Curve for Translation
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5 
    
    x_traj = x0 + s * (TARGET_X - x0)
    y_traj = y0 + s * (TARGET_Y - y0)
    z_traj = z0 + s * (TARGET_Z - z0)
    
    # SLERP for Rotation
    key_rots = R.from_quat([rot_start.as_quat(), rot_end.as_quat()])
    slerp = Slerp([0.0, 1.0], key_rots)
    quats = slerp(s).as_quat() 
    
    # ---------------------------------------------------------
    # 🛠️ THE MAGIC FIX: PREVENT TORQUE DISCONTINUITY
    # ---------------------------------------------------------
    # SciPy sometimes mathematically flips the angle by multiplying by -1.
    # We must check if the first line of our CSV matches the hardware!
    if np.dot(quats[0], start_quat_hardware) < 0:
        print("⚠️ Detected Quaternion flip from SciPy. Correcting automatically...")
        quats = -quats # Flip the whole trajectory back so the hardware stays happy
    # ---------------------------------------------------------
    
    # Safe Impedance Springs
    K_trans, K_rot = 600.0, 50.0
    D_trans, D_rot = 2.0 * np.sqrt(K_trans), 2.0 * np.sqrt(K_rot)
    
    print("3. Saving to 'quintic_trajectory.csv'...")
    data = []
    for i in range(steps):
        qx, qy, qz, qw = quats[i]
        data.append([
            x_traj[i], y_traj[i], z_traj[i], qw, qx, qy, qz, # Remember C++ needs qw first!
            K_trans, K_trans, K_trans, K_rot, K_rot, K_rot,
            D_trans, D_trans, D_trans, D_rot, D_rot, D_rot
        ])
        
    cols = ['x','y','z','qw','qx','qy','qz','Kx','Ky','Kz','Kax','Kay','Kaz','Dx','Dy','Dz','Dax','Day','Daz']
    df = pd.DataFrame(data, columns=cols)
    df.to_csv("quintic_trajectory.csv", index=False)
    print("✅ CSV Generated! DO NOT TOUCH THE ROBOT.")
    print("👉 Run: ./6d_pos_ori_stiff quintic_trajectory.csv")

if __name__ == "__main__":
    main()