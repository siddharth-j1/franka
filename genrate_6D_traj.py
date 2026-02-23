import numpy as np
import pandas as pd
from franky import Robot
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

def main():
    print("1. Connecting to robot to read safe starting pose...")
    try:
        robot = Robot("192.168.1.12")
        pose = robot.current_cartesian_state.pose.end_effector_pose
    except Exception as e:
        print(f"Connection error: {e}")
        return
        
    x0, y0, z0 = pose.translation
    qx0, qy0, qz0, qw0 = pose.quaternion
    rot_start = R.from_quat([qx0, qy0, qz0, qw0])
    
    # ---------------------------------------------------------
    # 15-SECOND TARGET PLAN (SLOW & OBSERVABLE)
    # ---------------------------------------------------------
    print("2. Planning 15-Second 6D Move (Translation + 90-Deg Twist)...")
    
    # Position: Move 10cm forward (X) and 5cm up (Z)
    # ---------------------------------------------------------
    # MAKE IT VISIBLE: 30cm Sideways Sweep
    # ---------------------------------------------------------
    # Position: Move 30cm sideways (Y) and 10cm up (Z). 
    # X stays the same to avoid stretching the arm too far!
    x1, y1, z1 = x0, y0 + 0.30, z0 + 0.10
    
    # Orientation: Twist the wrist exactly 90 degrees around its Z-axis
    rot_twist = R.from_euler('z', 90, degrees=True)
    rot_end = rot_start * rot_twist
    
    # Time settings
    T = 10.0  # Increased from 6s to 10s for a more gradual S-curve!
    hz = 1000
    steps = int(T * hz)
    tau = np.linspace(0, 1, steps)
    
    # Quintic smoothing (Zero velocity/accel at start and end)
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5 
    
    x_traj = x0 + s * (x1 - x0)
    y_traj = y0 + s * (y1 - y0)
    z_traj = z0 + s * (z1 - z0)
    
    # SLERP for perfectly smooth quaternion rotation
    key_rots = R.from_quat([rot_start.as_quat(), rot_end.as_quat()])
    slerp = Slerp([0.0, 1.0], key_rots)
    interp_rots = slerp(s)
    quats = interp_rots.as_quat() 
    
    # ---------------------------------------------------------
    # SAFE CRITICAL DAMPING
    # ---------------------------------------------------------
    K_trans, K_rot = 600.0, 50.0
    # Adjusted to 2.0 for Critical Damping (Zero bounce/oscillation)
    D_trans = 2.0 * np.sqrt(K_trans)
    D_rot   = 2.0 * np.sqrt(K_rot)
    
    print("3. Saving to 'quintic_trajectory.csv'...")
    data = []
    for i in range(steps):
        qx, qy, qz, qw = quats[i]
        data.append([
            x_traj[i], y_traj[i], z_traj[i], qw, qx, qy, qz,
            K_trans, K_trans, K_trans, K_rot, K_rot, K_rot,
            D_trans, D_trans, D_trans, D_rot, D_rot, D_rot
        ])
        
    cols = ['x','y','z','qw','qx','qy','qz','Kx','Ky','Kz','Kax','Kay','Kaz','Dx','Dy','Dz','Dax','Day','Daz']
    df = pd.DataFrame(data, columns=cols)
    df.to_csv("quintic_trajectory.csv", index=False)
    print(" 'quintic_trajectory.csv' successfully generated!")

if __name__ == "__main__":
    main()