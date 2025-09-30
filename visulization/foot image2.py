import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

# 필터링된 좌표 불러오기
coords_v4 = np.loadtxt("insole_outline_px_filtered.csv", delimiter=",")

# 폐곡선 보장
if not np.allclose(coords_v4[0], coords_v4[-1]):
    coords_v4 = np.vstack([coords_v4, coords_v4[0]])

# 스플라인 보간 (s 값 크게 해서 매끈하게)
tck, u = splprep([coords_v4[:,0], coords_v4[:,1]], s=2000.0, per=True)  # s ↑ = smoother
unew = np.linspace(0, 1, 1000)  # 더 촘촘하게 샘플링
xnew, ynew = splev(unew, tck)
coords_smooth = np.vstack([xnew, ynew]).T

# 저장
output_file_smooth = "insole_outline_px_smooth.csv"
np.savetxt(output_file_smooth, coords_smooth, delimiter=",", fmt="%.2f")

# 시각화
plt.figure(figsize=(5,10))
plt.plot(coords_v4[:,0], coords_v4[:,1], 'r.', alpha=0.3, label="Filtered Points")
plt.plot(xnew, ynew, 'b-', linewidth=2, label="Smoothed (s=2000)")
plt.gca().invert_yaxis()
plt.gca().set_aspect("equal")
plt.legend()
plt.title("Insole Outline - Smoothed v2")
plt.show()

print("저장 완료:", output_file_smooth, coords_smooth.shape)
