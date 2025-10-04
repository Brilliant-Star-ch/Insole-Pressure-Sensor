import numpy as np
import matplotlib.pyplot as plt
import glob

# ===== Load insole outline (smoothed) =====
outline_file = "visualization/insole outline/insole_outline_px_smooth.csv"
outline = np.loadtxt(outline_file, delimiter=",")

# 좌우 대칭: 오른발 윤곽선 기준, x축을 이미지 중심에서 반전
x_center = outline[:,0].mean()
outline_left = outline.copy()
outline_left[:,0] = 2*x_center - outline_left[:,0]

# 왼발 아웃라인의 x좌표 최대값 계산
left_max_x = np.max(outline_left[:,0])
# 오른발 아웃라인을 300 만큼 떨어지게 이동
offset = 300
outline_right_shifted = outline.copy()
outline_right_shifted[:,0] = outline_right_shifted[:,0] + offset

# 센서 위치도 동일하게 이동(백업용)
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
data_files = glob.glob("visualization/insole data/*.csv")
if not data_files:
    raise FileNotFoundError("No CSV files found in 'visualization/insole data' folder.")

data = np.loadtxt(data_files[0], delimiter=",", skiprows=2)
channels_right = data[:,2:18]  # 오른발 16채널
channels_left  = data[:,18:34] # 왼발 16채널

# ===== Visualization (Frame 선택) =====
frame_idx = 90  # 원하는 시간 프레임 인덱스

plt.figure(figsize=(12, 8))
# 왼발 윤곽선 (실선)
plt.plot(outline_left[:,0], outline_left[:,1], 'k-', label="Left Outline")
# 오른발 윤곽선 (실선, 오른쪽으로 이동)
plt.plot(outline_right_shifted[:,0], outline_right_shifted[:,1], 'k-.', label="Right Outline")

# 센서값 색상: 'inferno' 컬러맵 사용 (흰색 없음, 어두운 계열)
cmap_choice = 'inferno'

# 센서값을 그대로 사용 (4095=어두운색, 0=밝은색)
right_colors = channels_right[frame_idx]
left_colors = channels_left[frame_idx]

# 왼발 센서값
sc_l = plt.scatter(sensor_pos_left[:,0], sensor_pos_left[:,1], c=left_colors, cmap=cmap_choice, s=200, vmin=0, vmax=4095, edgecolors='black', marker='s', label="Left Sensors")
# 오른발 센서값 (오른쪽으로 이동)
sc_r = plt.scatter(sensor_pos_right[:,0], sensor_pos_right[:,1], c=right_colors, cmap=cmap_choice, s=200, vmin=0, vmax=4095, edgecolors='black', label="Right Sensors")

plt.title(f"Both Feet Sensor Visualization (Frame {frame_idx})")
plt.gca().invert_yaxis()
plt.gca().set_aspect("equal")
plt.legend()
plt.colorbar(sc_r, label="Sensor Value (0=bright, 4095=dark)")
plt.tight_layout()
plt.show()