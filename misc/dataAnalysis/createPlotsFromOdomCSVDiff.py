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

# Create the r2_r4_df DataFrame with the differences
r2_r4_df = pd.DataFrame()
r2_r4_df['x_diff'] = r2_df['x_r2'] - r4_df['x_r4']
r2_r4_df['y_diff'] = r2_df['y_r2'] - r4_df['y_r4']

# Calculate the Euclidean distance and store it in a numpy array
euclidean_distance = np.sqrt(r2_r4_df['x_diff']**2 + r2_r4_df['y_diff']**2).to_numpy()

# Convert columns to numpy arrays
time = r4_df['time'].to_numpy() / 1e9

# Find the index where the data changes from 0
change_indices = np.where(euclidean_distance > 0)[0]

# If there are no changes, just keep the last 3 seconds of data
if len(change_indices) == 0:
    index_3_seconds = max(len(time) - int(3 / (time[1] - time[0])), 0)
else:
    # Find the first change index and keep the data preceding it by 3 seconds
    index_3_seconds = max(change_indices[0] - int(3 / (time[1] - time[0])), 0)

time = time[index_3_seconds:] - time[index_3_seconds] #-initial_time
euclidean_distance = euclidean_distance[index_3_seconds:]

# Ensure that both arrays have the same length
min_length = min(len(time), len(euclidean_distance))
time = time[:min_length] # cut it off
euclidean_distance = euclidean_distance[:min_length]

#@TODO Convert time from nanoseconds to seconds and

font = {'family': 'DejaVu Sans',
        'weight': 'bold',
        'size': 12}

plt.rc('font', **font)

plt.plot(time, euclidean_distance)
plt.xlabel('Time (s)')
plt.ylabel('Diff distance from initial positions (m)')
plt.title('Difference from Initial Positions Over Time')
plt.legend()
plt.grid(True)
plt.savefig('diff_plot.png')

