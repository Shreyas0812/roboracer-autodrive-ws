import os
import pandas as pd

# Define file paths
input_path = os.path.join('backup_waypoints', 'traj_race_cl_mincurv.csv')
output_path = os.path.join('config', 'waypoints.csv')

# Read the CSV file, skipping comment lines and using ';' as the separator
df = pd.read_csv(
    input_path,
    sep=';',
    comment='#',
    header=None,
    engine='python'
)

# Drop the first column (index 0)
df = df.drop(columns=[0])

# Strip whitespace and ensure all values are floats
df = df.applymap(lambda x: float(str(x).strip()))

# Save to output.csv with comma as separator, no header or index
df.to_csv(output_path, sep=',', header=False, index=False, float_format='%.7f')

print(f"Conversion complete! Output written to {output_path}")
