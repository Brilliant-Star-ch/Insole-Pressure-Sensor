
import serial, time, sys, random, os
import numpy as np
import matplotlib.pyplot as plt

# =====================
# User Config
# =====================
PORT = "COM5"
BAUD = 115200
ADC_MAX = 4095

# Expected serial format:
#   time_ms,ch0,ch1,...,ch31   (33 items total)
#   * If only 16 values are provided (legacy), left foot will be zero-filled.
#
# Channel mapping assumption (change if your MCU sends opposite):
#   ch0..ch15  -> RIGHT foot (16 sensors)
#   ch16..ch31 -> LEFT  foot (16 sensors)
#
# Coordinate system used for plotting (normalized):
W = 100.0   # width
H = 260.0   # height

# ---------------------------------------
# Sensor coordinates (RIGHT foot layout)
#   Units: normalized to 0..W (x), 0..H (y)
#   Tune these points to match your physical sensor placement.
# ---------------------------------------
SENSOR_COORDS_RIGHT = [
    (68,  30),  # ch0  : heel outer
    (50,  30),  # ch1  : heel inner
    (40,  75),  # ch2  : mid inner
    (60,  75),  # ch3  : mid outer
    (45, 110),  # ch4  : mid inner-fore
    (62, 110),  # ch5  : mid outer-fore
    (40, 145),  # ch6  : fore inner
    (60, 145),  # ch7  : fore outer
    (35, 170),  # ch8  : fore inner 2
    (63, 170),  # ch9  : fore outer 2
    (30, 195),  # ch10 : toes inner
    (70, 195),  # ch11 : toes outer
    (28, 215),  # ch12 : big toe base
    (62, 215),  # ch13 : small toes base
    (38, 235),  # ch14 : big toe mid
    (57, 235),  # ch15 : small toe mid
]

def mirror_left(coords):
    """Mirror RIGHT-foot coordinates to LEFT across vertical midline (x' = W - x)."""
    return [(W - x, y) for (x, y) in coords]

SENSOR_COORDS_LEFT = mirror_left(SENSOR_COORDS_RIGHT)

# =====================
# Helpers
# =====================
def load_outline_csv(path="insole_outline_px_smooth.csv"):
    """Load outline CSV (pixel coords), normalize to plotting space (W,H), flip Y."""
    if not os.path.exists(path):
        print(f"[WARN] Outline file not found: {path}. Outline will be skipped.")
        return None
    try:
        arr = np.loadtxt(path, delimiter=",")
        if arr.ndim != 2 or arr.shape[1] < 2:
            print(f"[WARN] Unexpected outline shape: {arr.shape}. Outline will be skipped.")
            return None
        # Normalize outline to 0..W and 0..H by bounding box
        min_x, max_x = np.min(arr[:,0]), np.max(arr[:,0])
        min_y, max_y = np.min(arr[:,1]), np.max(arr[:,1])
        sx = (arr[:,0] - min_x) / (max_x - min_x + 1e-9) * W
        sy = (arr[:,1] - min_y) / (max_y - min_y + 1e-9) * H
        # Flip Y (so origin at bottom-left visually)
        sy = H - sy
        return np.column_stack([sx, sy])
    except Exception as e:
        print(f"[WARN] Failed to load outline CSV: {e}. Outline will be skipped.")
        return None

def parse_line(line: str):
    """Parse 'time_ms,ch0,...' -> (t:int, list_of_values). Accept 16 or 32 channels."""
    parts = line.strip().split(',')
    try:
        # basic guard
        if len(parts) < 2:
            return None, None
        t = int(parts[0])
        vals = list(map(int, parts[1:]))
        if len(vals) == 16:
            # legacy: pad to 32 with zeros for left foot
            vals = vals + [0]*16
        elif len(vals) == 32:
            pass
        else:
            # unexpected length
            return None, None
        return t, vals
    except:
        return None, None

def get_fake_vals(nch=32):
    return int(time.time()*1000), [random.randint(400, 3200) for _ in range(nch)]

# =====================
# Main
# =====================
def main():
    outline = load_outline_csv("insole_outline_px_smooth.csv")
    outline_left = None
    outline_right = None
    if outline is not None:
        # For RIGHT: use as-is
        outline_right = outline.copy()
        # For LEFT: mirror X (x' = W - x)
        outline_left = outline.copy()
        outline_left[:,0] = W - outline_left[:,0]

    # Open serial (fallback to FAKE mode if fails)
    use_fake = False
    ser = None
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"Connected to {PORT} @ {BAUD}")
    except Exception as e:
        print(f"[INFO] Serial open failed: {e} -> FAKE mode")
        use_fake = True

    # Figure: two panels (LEFT, RIGHT)
    plt.ion()
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 6.0))
    axL, axR = axes

    for ax, title in zip((axL, axR), ("Left Foot", "Right Foot")):
        ax.set_xlim(0, W)
        ax.set_ylim(0, H)
        ax.set_aspect('equal')
        ax.set_xticks([]); ax.set_yticks([])
        ax.set_title(title)

    # Draw outlines
    if outline_left is not None:
        axL.plot(outline_left[:,0], outline_left[:,1], 'k-', linewidth=1.6, alpha=0.85)
    if outline_right is not None:
        axR.plot(outline_right[:,0], outline_right[:,1], 'k-', linewidth=1.6, alpha=0.85)

    # Sensor scatters
    xsL = [p[0] for p in SENSOR_COORDS_LEFT]
    ysL = [p[1] for p in SENSOR_COORDS_LEFT]
    xsR = [p[0] for p in SENSOR_COORDS_RIGHT]
    ysR = [p[1] for p in SENSOR_COORDS_RIGHT]

    valsL = [0]*16
    valsR = [0]*16

    scatL = axL.scatter(xsL, ysL, c=valsL, cmap='hot', s=520, vmin=0, vmax=ADC_MAX, edgecolors='k', linewidths=0.5)
    scatR = axR.scatter(xsR, ysR, c=valsR, cmap='hot', s=520, vmin=0, vmax=ADC_MAX, edgecolors='k', linewidths=0.5)

    cbarR = plt.colorbar(scatR, ax=axR, pad=0.02)
    cbarR.set_label("ADC")
    # Optional: one shared colorbar could be implemented, but separate is fine for clarity.

    # --- Optional: read & skip header / prefetch first data ---
    first_data = None
    if not use_fake:
        start = time.time()
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                if time.time() - start > 3.0:
                    print("[INFO] No serial data -> FAKE mode")
                    use_fake = True
                    break
                continue
            if line.startswith("time_ms"):
                print("[INFO] Header detected, skip it")
                break
            else:
                t, v = parse_line(line)
                if v is not None:
                    first_data = (t, v)
                    print("[INFO] No header found, using first data line")
                    break

    # --- Update loop ---
    try:
        if first_data:
            t, v = first_data
            valsR = v[0:16]
            valsL = v[16:32]
            scatR.set_array(np.array(valsR))
            scatL.set_array(np.array(valsL))
            fig.suptitle(f"Insole Pressure  t={t} ms")
            plt.pause(0.05)

        while True:
            if use_fake:
                t, v = get_fake_vals(32)
            else:
                line = ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                t, v = parse_line(line)
                if v is None:
                    continue

            valsR = v[0:16]
            valsL = v[16:32]

            scatR.set_array(np.array(valsR))
            scatL.set_array(np.array(valsL))

            fig.suptitle(f"Insole Pressure  t={t} ms")
            plt.pause(0.04)

    except KeyboardInterrupt:
        pass
    finally:
        if ser:
            ser.close()
        print("Closed")

if __name__ == "__main__":
    main()
