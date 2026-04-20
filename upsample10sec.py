import pandas as pd
import numpy as np

# Your current fast CSV
INPUT_CSV = "exp3b_trajectory.csv"
# The new, slow 10-second CSV
OUTPUT_CSV = "exp3a_trajectory_10sec.csv"

# How many times to repeat each row
MULTIPLIER = 10 

def main():
    print(f"Reading original file: {INPUT_CSV}")
    try:
        df = pd.read_csv(INPUT_CSV)
    except Exception as e:
        print(f"❌ Failed to load: {e}")
        return
        
    original_length = len(df)
    print(f"Original length: {original_length} rows ({original_length / 1000.0} seconds)")

    # The Pandas "repeat" trick: Duplicates each row MULTIPLIER times
    # reset_index() renumbers them cleanly
    df_slow = df.loc[df.index.repeat(MULTIPLIER)].reset_index(drop=True)

    new_length = len(df_slow)
    print(f"Upsampled length: {new_length} rows ({new_length / 1000.0} seconds)")

    print(f"Saving to {OUTPUT_CSV}...")
    df_slow.to_csv(OUTPUT_CSV, index=False)
    print("✅ Done!")

if __name__ == "__main__":
    main()