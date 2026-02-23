import numpy as np

def main():
    HZ = 1000
    duration = 5.0
    steps = int(duration * HZ)
    
    print(f"Generating {steps} rows of Dummy AI Data...")
    
    # 1. Open a raw text/csv file
    with open('dummy_trajectory.csv', 'w') as f:
        # Write header for clarity (21 columns total)
        f.write("dx,dy,dz,k11,k12,k13,k21,k22,k23,k31,k32,k33,d11,d12,d13,d21,d22,d23,d31,d32,d33\n")
        
        # 2. Fixed Dummy Impedance (200 N/m)
        K = np.diag([200.0, 200.0, 200.0])
        D = np.diag([28.0, 28.0, 28.0])
        
        for t in range(steps):
            # S-Curve for 5cm slide in X
            percent = t / steps
            s = 10*(percent**3) - 15*(percent**4) + 6*(percent**5)
            dx = s * 0.05
            dy = 0.0
            dz = 0.0
            
            # Flatten the 3x3 matrices into a single row of text
            k_str = ",".join(map(str, K.flatten()))
            d_str = ",".join(map(str, D.flatten()))
            
            # Write exactly one millisecond of data
            f.write(f"{dx},{dy},{dz},{k_str},{d_str}\n")
            
    print("Success! Saved 'dummy_trajectory.csv'")

if __name__ == "__main__":
    main()