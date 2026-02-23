import numpy as np
import pandas as pd
from franky import Robot

def main():
    print("1. Connecting to robot to read safe starting pose...")
    try:
        robot = Robot("192.168.1.12")
        pose = robot.current_cartesian_state.pose.end_effector_pose
    except Exception as e:
        print(f" Could not connect to robot: {e}")
        return
        
    x, y, z = pose.translation
    qx, qy, qz, qw = pose.quaternion # Corrected order!
    
    # ---------------------------------------------------------
    # THE DRAMATIC 30-SECOND DECAY PROFILE
    # ---------------------------------------------------------
    hz = 1000
    
    # Phase 1: Hold Stiff for 5 seconds (K = 1000)
    K_stiff = np.full(5 * hz, 1000.0)
    
    # Phase 2: Decay smoothly over 10 seconds (1000 down to 50)
    K_decay = np.linspace(1000.0, 50.0, 10 * hz)
    
    # Phase 3: Hold Soft for 15 seconds (K = 50)
    K_soft = np.full(15 * hz, 50.0)
    
    # Combine into a 30-second timeline
    K_trans = np.concatenate([K_stiff, K_decay, K_soft])
    
    # Rotational stiffness: Decay it slightly too (50 down to 15) so it doesn't fight your wrist
    K_rot = np.concatenate([
        np.full(5 * hz, 50.0),
        np.linspace(50.0, 15.0, 10 * hz),
        np.full(15 * hz, 15.0)
    ])
    
    # Critical Damping: D = 2 * sqrt(K)
    D_trans = 2.0 * np.sqrt(K_trans)
    D_rot = 2.0 * np.sqrt(K_rot)
    
    steps = len(K_trans)
    
    print(f"2. Generating {steps/1000} seconds of decay data...")
    data = []
    for i in range(steps):
        row = [
            x, y, z, qw, qx, qy, qz,
            K_trans[i], K_trans[i], K_trans[i], K_rot[i], K_rot[i], K_rot[i],
            D_trans[i], D_trans[i], D_trans[i], D_rot[i], D_rot[i], D_rot[i]
        ]
        data.append(row)
        
    cols = ['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz', 
            'Kx', 'Ky', 'Kz', 'Kax', 'Kay', 'Kaz', 
            'Dx', 'Dy', 'Dz', 'Dax', 'Day', 'Daz']
    
    df = pd.DataFrame(data, columns=cols)
    df.to_csv("decay_trajectory.csv", index=False)
    print("'decay_trajectory.csv' generated successfully!")

if __name__ == "__main__":
    main()