import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

# Load filtered coordinates
coords_v4 = np.loadtxt("visualization/insole outline/insole_outline_px_filtered.csv", delimiter=",")

# Ensure closed curve
if not np.allclose(coords_v4[0], coords_v4[-1]):
    coords_v4 = np.vstack([coords_v4, coords_v4[0]])

# Spline interpolation (high s for smoothness)
tck, u = splprep([coords_v4[:,0], coords_v4[:,1]], s=2000.0, per=True)  # s ↑ = smoother
unew = np.linspace(0, 1, 1000)  # Dense sampling
xnew, ynew = splev(unew, tck)
coords_smooth = np.vstack([xnew, ynew]).T

# Save result
output_file_smooth = "visualization/insole outline/insole_outline_px_smooth.csv"
np.savetxt(output_file_smooth, coords_smooth, delimiter=",", fmt="%.2f")

# Visualization
plt.figure(figsize=(5,10))
plt.plot(coords_v4[:,0], coords_v4[:,1], 'r.', alpha=0.3, label="Filtered Points")
plt.plot(xnew, ynew, 'b-', linewidth=2, label="Smoothed (s=2000)")
plt.gca().invert_yaxis()
plt.gca().set_aspect("equal")
plt.legend()
plt.title("Insole Outline - Smoothed v2")
plt.show()

print("Save complete:", output_file_smooth, coords_smooth.shape)
