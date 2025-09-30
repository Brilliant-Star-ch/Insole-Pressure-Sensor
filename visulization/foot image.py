import cv2
import numpy as np
import matplotlib.pyplot as plt

# ===== 사용자 설정 =====
IMG_FILE = "insole2.png"   # 분석할 이미지 파일
OUT_CSV  = "insole_outline_px.csv"  # 저장할 좌표 파일 (픽셀 단위)

# ===== 1. 이미지 불러오기 =====
img = cv2.imread(IMG_FILE, cv2.IMREAD_GRAYSCALE)
if img is None:
    raise FileNotFoundError(f"{IMG_FILE} 파일을 같은 폴더에 넣어주세요!")

h, w = img.shape
print(f"이미지 크기: {w}x{h} px")

# ===== 2. 이진화 =====
_, thresh = cv2.threshold(img, 90, 255, cv2.THRESH_BINARY_INV)

# ===== 3. 윤곽선 추출 =====
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

if len(contours) == 0:
    raise RuntimeError("윤곽선을 찾지 못했습니다. threshold 값을 조정하세요.")

# 가장 큰 윤곽선 선택
contour = max(contours, key=cv2.contourArea)
coords_px = contour.reshape(-1, 2)  # (N,2)

print(f"윤곽 좌표 개수: {len(coords_px)}")

# ===== 4. CSV 저장 =====
np.savetxt(OUT_CSV, coords_px, delimiter=",", fmt="%d")
print(f"좌표 {OUT_CSV} 저장 완료 (픽셀 단위)")

# ===== 5. 시각화 =====
plt.figure(figsize=(4,8))
plt.imshow(img, cmap="gray")
plt.plot(coords_px[:,0], coords_px[:,1], 'r-', linewidth=2, label="Pixel Outline")
plt.title("Insole Outline (Pixel Coordinates)")
plt.legend()
plt.gca().invert_yaxis()  # 이미지 좌표계와 맞추려면 유지
plt.show()
