import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read the CSV files into pandas DataFrames
r2_df = pd.read_csv('r2_odom.csv')
r4_df = pd.read_csv('r4_odom.csv')

# Check if DataFrames are empty
if r2_df.empty or r4_df.empty:
    raise ValueError("CSV files are empty")

# Select the columns you need and rename them
r2_df = r2_df[['%time', 'field.pose.pose.position.x', 'field.pose.pose.position.y']]
r2_df.columns = ['time', 'x_r2', 'y_r2']

r4_df = r4_df[['%time', 'field.pose.pose.position.x', 'field.pose.pose.position.y']]
r4_df.columns = ['time', 'x_r4', 'y_r4']

# Convert time from nanoseconds to seconds
r2_df['time'] = r2_df['time'] * 1e-9
r4_df['time'] = r4_df['time'] * 1e-9

# Get the initial values of x and y positions for both robots
x_init_r2 = r2_df['x_r2'].iloc[0]
y_init_r2 = r2_df['y_r2'].iloc[0]

x_init_r4 = r4_df['x_r4'].iloc[0]
y_init_r4 = r4_df['y_r4'].iloc[0]

# Subtract the initial values from x positions
r2_df['x_r2'] = r2_df['x_r2'] - x_init_r2
r4_df['x_r4'] = r4_df['x_r4'] - x_init_r4

# Subtract the initial values from y positions
r2_df['y_r2'] = r2_df['y_r2'] - y_init_r2
r4_df['y_r4'] = r4_df['y_r4'] - y_init_r4

# Calculate the distance for both robots
r2_df['distance_r2'] = np.sqrt(r2_df['x_r2']**2 + r2_df['y_r2']**2)
r4_df['distance_r4'] = np.sqrt(r4_df['x_r4']**2 + r4_df['y_r4']**2)

# Find the index where the distance changes for r2 and r4
change_index_r2 = np.where(r2_df['distance_r2'].diff() > 0)[0]
change_index_r4 = np.where(r4_df['distance_r4'].diff() > 0)[0]

# Get the time value where the distance changes for r2 and r4
if len(change_index_r2) > 0:
    time_change_r2 = r2_df['time'].iloc[change_index_r2[0]]
else:
    time_change_r2 = float('inf')

if len(change_index_r4) > 0:
    time_change_r4 = r4_df['time'].iloc[change_index_r4[0]]
else:
    time_change_r4 = float('inf')

# Calculate the time to keep for 3 seconds of initial 0s
one_second = 3  # 3 seconds
start_time = min(time_change_r2, time_change_r4) - one_second
end_time = min(max(r2_df['time']), max(r4_df['time']))

# Filter the data to keep only the desired time range
r2_df = r2_df[(r2_df['time'] >= start_time) & (r2_df['time'] <= end_time)]
r4_df = r4_df[(r4_df['time'] >= start_time) & (r4_df['time'] <= end_time)]

# Shift the time values to start from 0
r2_df['time'] = r2_df['time'] - start_time
r4_df['time'] = r4_df['time'] - start_time

# Convert columns to numpy arrays
time_r2 = r2_df['time'].to_numpy()
distance_r2 = r2_df['distance_r2'].to_numpy()

time_r4 = r4_df['time'].to_numpy()
distance_r4 = r4_df['distance_r4'].to_numpy()

font = {'family' : 'DejaVu Sans',
        'weight' : 'bold',
        'size'   : 12}

plt.rc('font', **font)

plt.plot(time_r2, distance_r2, label='r2')
plt.plot(time_r4, distance_r4, label='r4')
plt.xlabel('Time (s)')
plt.ylabel('Distance from Initial Position (m)')
plt.title('Distance from Initial Position Over Time')
plt.legend()
plt.grid(True)
plt.savefig('Odom.png')

