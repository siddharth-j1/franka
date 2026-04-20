import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

def main():
    # Allow passing custom target CSV, default to our latest test
    target_csv = sys.argv[1] if len(sys.argv) > 1 else "quintic_trajectory.csv"
    log_csv = "execution_log.csv"

    print(f"Loading Planned Data: {target_csv}")
    print(f"Loading Actual Data:  {log_csv}")
    
    try:
        planned = pd.read_csv(target_csv)
        actual = pd.read_csv(log_csv)
    except FileNotFoundError as e:
        print(f"Error: {e}. Did you run the C++ execution step first?")
        return

    # Ensure lengths match in case the robot stopped a few milliseconds early
    min_len = min(len(planned), len(actual))
    planned = planned.iloc[:min_len]
    actual = actual.iloc[:min_len]

    # Convert rows to seconds for the X-axis (1000Hz = 1ms per row)
    time = np.linspace(0, min_len / 1000.0, min_len)

    # ---------------------------------------------------------
    # PLOT 1: X, Y, Z Position Tracking
    # ---------------------------------------------------------
    fig1, axes1 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig1.suptitle("Cartesian Position Tracking (Planned vs Actual)", fontsize=14)
    
    # Plot X
    axes1[0].plot(time, planned['x'], label='Planned X', color='blue', linewidth=2)
    axes1[0].plot(time, actual['x'], label='Actual X', color='cyan', linestyle='dashed', linewidth=2)
    axes1[0].set_ylabel('X (meters)')
    axes1[0].legend(loc="upper right")
    axes1[0].grid(True)

    # Plot Y
    axes1[1].plot(time, planned['y'], label='Planned Y', color='red', linewidth=2)
    axes1[1].plot(time, actual['y'], label='Actual Y', color='orange', linestyle='dashed', linewidth=2)
    axes1[1].set_ylabel('Y (meters)')
    axes1[1].legend(loc="upper right")
    axes1[1].grid(True)

    # Plot Z
    axes1[2].plot(time, planned['z'], label='Planned Z', color='green', linewidth=2)
    axes1[2].plot(time, actual['z'], label='Actual Z', color='lightgreen', linestyle='dashed', linewidth=2)
    axes1[2].set_ylabel('Z (meters)')
    axes1[2].set_xlabel('Time (seconds)')
    axes1[2].legend(loc="upper right")
    axes1[2].grid(True)

    plt.tight_layout()
    fig1.savefig("tracking_position.png")

    # ---------------------------------------------------------
    # PLOT 2: Orientation Tracking (Quaternions)
    # ---------------------------------------------------------
    fig2, axes2 = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    fig2.suptitle("Orientation Tracking (Quaternions)", fontsize=14)

    quats = ['qw', 'qx', 'qy', 'qz']
    colors = ['purple', 'brown', 'pink', 'gray']

    for i, q in enumerate(quats):
        axes2[i].plot(time, planned[q], label=f'Planned {q.upper()}', color=colors[i], linewidth=2)
        axes2[i].plot(time, actual[q], label=f'Actual {q.upper()}', color='black', linestyle='dotted', linewidth=2)
        axes2[i].set_ylabel(q.upper())
        axes2[i].legend(loc="upper right")
        axes2[i].grid(True)

    axes2[3].set_xlabel('Time (seconds)')
    plt.tight_layout()
    fig2.savefig("tracking_orientation.png")

    print("Plots generated! Saved as 'tracking_position.png' and 'tracking_orientation.png'.")
    plt.show()

if __name__ == "__main__":
    main()