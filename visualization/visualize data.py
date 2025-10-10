import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import glob
import time
import re

plt.ion()  # interactive mode ON

# ===== Load insole outline (smoothed) =====
outline_file = "visualization/insole outline/insole_outline_px_smooth.csv"
outline = np.loadtxt(outline_file, delimiter=",")

# Left and right symmetry: Right foot contour, invert the x-axis from the center of the image
x_center = outline[:,0].mean()
outline_left = outline.copy()
outline_left[:,0] = 2*x_center - outline_left[:,0]

# Calculate the x-coordinate maximum for the left foot outline
left_max_x = np.max(outline_left[:,0])
# Move the right foot outline down by 300
offset = 300
outline_right_shifted = outline.copy()
outline_right_shifted[:,0] = outline_right_shifted[:,0] + offset

# Sensor position moved equally
sensor_pos_right = np.array([
    [585, 180], [650, 180],
    [560, 290], [630, 290], [700, 290],
    [580, 385], [650, 385], [720, 385],
    [590, 480], [660, 480], [720, 480],
    [596, 600], [650, 600], [700, 600],
    [610, 760], [700, 760]
])
sensor_pos_left = sensor_pos_right.copy()
sensor_pos_left[:,0] = 2.86*x_center - sensor_pos_left[:,0]

# ===== Load sensor data =====
data_files = glob.glob("visualization/insole data/COMBO_LOG*.csv")
if not data_files:
    raise FileNotFoundError("No CSV files found in 'visualization/insole data' folder.")

# Extract and sort numbers from file names (e.g., COMBO_LOG001 → 1)
def extract_log_number(filename):
    match = re.search(r"COMBO_LOG(\d+)", filename)
    return int(match.group(1)) if match else -1

# Select the file with the highest number
data_files.sort(key=extract_log_number)
latest_file = data_files[-1]

print(f"[INFO] Using latest data file: {latest_file}")

data = np.loadtxt(data_files[0], delimiter=",", skiprows=2)
frame_id = data[:, 1].astype(int)
flag = data[:, 34].astype(int)
channels_right = data[:,2:18]  # Right 16channels
channels_left  = data[:,18:34] # Left 16channels

# ===== Visualization settings =====
cmap_choice = 'RdBu_r'
fig, ax = plt.subplots(figsize=(12, 8))

# Marker size scaling function
def scale_marker_size(values, min_size=100, max_size=500):
    vmin, vmax = 0, 4095
    scaled = max_size -  (values - vmin) / (vmax - vmin) * (max_size - min_size)
    return scaled

# ===== Colorbar Settings =====
sc = ax.scatter([], [], c=[], cmap=cmap_choice, vmin=0, vmax=4095)
cbar = plt.colorbar(sc, ax=ax, label="(Blue = Low, Red = High)")

# delete colorbar ticks and labels
cbar.set_ticks([]) # Delete tickts
cbar.set_label('') # Delete Label

for i in range(len(frame_id)):
    if flag[i] != 0:
        continue

    ax.clear()

    # Redraw outline
    ax.plot(outline_left[:, 0], outline_left[:, 1], 'k-')
    ax.plot(outline_right_shifted[:, 0], outline_right_shifted[:, 1], 'k-')

    # Fill left foot outline
    ax.fill(outline_left[:, 0], outline_left[:, 1],
            color='#FFE0BD', alpha=0.5, label="Left Foot")

    # Fill right foot outline
    ax.fill(outline_right_shifted[:, 0], outline_right_shifted[:, 1],
            color='#FFE0BD', alpha=0.5, label="Right Foot")

    right_colors = channels_right[i]
    left_colors = channels_left[i]
    right_sizes = scale_marker_size(right_colors)
    left_sizes = scale_marker_size(left_colors)

    # Draw left/right foot sensors
    ax.scatter(sensor_pos_left[:, 0], sensor_pos_left[:, 1],
               c=4095-left_colors, cmap=cmap_choice, s=left_sizes,
               vmin=0, vmax=4095, edgecolors='black')

    ax.scatter(sensor_pos_right[:, 0], sensor_pos_right[:, 1],
               c=4095-right_colors, cmap=cmap_choice, s=right_sizes,
               vmin=0, vmax=4095, edgecolors='black')

    ax.set_title(f"Frame ID: {frame_id[i]}")
    ax.invert_yaxis()
    ax.set_aspect("equal")

    plt.pause(0.05)  # 50ms per frame

plt.ioff()
plt.show()