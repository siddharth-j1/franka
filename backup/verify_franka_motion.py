import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def main():
    print("Loading Planned CSV and Physical Execution CSV...")
    
    # 1. Load the Robot's actual recorded path (Absolute Coordinates)
    try:
        real_data = pd.read_csv('execution_log.csv')
        real_pos = real_data[['real_x', 'real_y', 'real_z']].values
    except FileNotFoundError:
        print(" Error: 'execution_log.csv' not found. Run the C++ code first!")
        return

    # 2. Load the AI's intended path (Relative Offsets)
    try:
        plan_data = pd.read_csv('real_trajectory.csv')
        plan_dx_dy_dz = plan_data[['dx', 'dy', 'dz']].values
    except FileNotFoundError:
        print("Error: 'real_trajectory.csv' not found.")
        return

    # Ensure lengths match in case the C++ loop finished a frame early
    min_len = min(len(real_pos), len(plan_dx_dy_dz))
    real_pos = real_pos[:min_len]
    plan_dx_dy_dz = plan_dx_dy_dz[:min_len]

    # ---------------------------------------------------------
    # 3. THE FIX: Clean the Data and Convert to Absolute Space
    # ---------------------------------------------------------
    # Delete the 0th frame because C++ left it as [0,0,0]
    real_pos = real_pos[1:]
    plan_dx_dy_dz = plan_dx_dy_dz[1:]

    # Re-calculate the length for the time axis
    clean_len = len(real_pos)
    t_axis = np.linspace(0, clean_len / 1000.0, clean_len)

    # Now grab the true physical starting location (which is now safely at index 0)
    start_pos = real_pos[0] 
    
    # Add the AI's mathematical offsets to it
    planned_pos = start_pos + plan_dx_dy_dz

    # 4. Calculate True Tracking Error (Euclidean distance in millimeters)
    tracking_error = np.linalg.norm(planned_pos - real_pos, axis=1) * 1000

    # ---------------------------------------------------------
    # 5. Plot the Verification Graphs for your Paper
    # ---------------------------------------------------------
    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    axis_labels = ['X Position (m)', 'Y Position (m)', 'Z Position (m)']
    
    for i in range(3):
        # We plot `planned_pos` here, NOT the raw `plan_dx_dy_dz`
        axes[i].plot(t_axis, planned_pos[:, i], 'b--', label='AI Plan (Absolute)', linewidth=2)
        axes[i].plot(t_axis, real_pos[:, i], 'r-', label='Real Hardware', linewidth=2)
        axes[i].set_ylabel(axis_labels[i])
        axes[i].legend(loc="upper right")
        axes[i].grid(True)
        
    axes[0].set_title("C-GMS Hardware Verification: AI Plan vs. Actual Execution")

    # Plot the True Error
    axes[3].plot(t_axis, tracking_error, 'k-', linewidth=2)
    axes[3].set_ylabel("Tracking Error (mm)")
    axes[3].set_xlabel("Time (seconds)")
    axes[3].grid(True)
    
    # Add a gray fill to make the error curve pop
    axes[3].fill_between(t_axis, tracking_error, color='gray', alpha=0.3)

    plt.tight_layout()
    plt.savefig('hardware_verification_fixed.png')
    print("Success! Open 'hardware_verification_fixed.png' to see the true results.")

if __name__ == "__main__":
    main()