import cv2
import numpy as np
import matplotlib.pyplot as plt

# ===== User Config =====
IMG_FILE = "visualization/insole image/insole2.png"   # Image file to analyze
OUT_CSV  = "visualization/insole outline/insole_outline_px.csv"      # Output coordinate file (pixel units)

# ===== 1. Load Image =====
img = cv2.imread(IMG_FILE, cv2.IMREAD_GRAYSCALE)
if img is None:
    raise FileNotFoundError(f"Please put {IMG_FILE} in the 'insole image' folder!")

h, w = img.shape
print(f"Image size: {w}x{h} px")

# ===== 2. Thresholding =====
_, thresh = cv2.threshold(img, 90, 255, cv2.THRESH_BINARY_INV)

# ===== 3. Find Contours =====
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

if len(contours) == 0:
    raise RuntimeError("No contours found. Try adjusting the threshold value.")

# Select the largest contour
contour = max(contours, key=cv2.contourArea)
coords_px = contour.reshape(-1, 2)  # (N,2)

print(f"Number of outline coordinates: {len(coords_px)}")

# ===== 4. Save CSV =====
np.savetxt(OUT_CSV, coords_px, delimiter=",", fmt="%d")
print(f"Coordinates saved to {OUT_CSV} (pixel units)")

# ===== 5. Visualization =====
plt.figure(figsize=(4,8))
plt.imshow(img, cmap="gray")
plt.plot(coords_px[:,0], coords_px[:,1], 'r-', linewidth=2, label="Pixel Outline")
plt.title("Insole Outline (Pixel Coordinates)")
plt.legend()
plt.gca().invert_yaxis()  # Keep to match image coordinate system
plt.show()
