import serial, time, sys, random
import numpy as np
import matplotlib.pyplot as plt

# ===== 사용자 설정 =====
PORT = "COM5"
BAUD = 115200
FOOT = "right"  # "right" or "left" (좌우 반전)

# 센서 채널 순서 → 좌표 매핑(채널 0~15가 어떤 좌표에 대응하는지)
# 사진을 보고 대략적으로 배치한 "오른발" 기준 좌표 (단위: 임의, 가로 0~100, 세로 0~260)
# 필요하면 값만 손봐주면 됨.
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

# 외곽선(오른발) 윤곽 좌표(대략) — 점들을 부드럽게 연결해서 그린다.
# 필요하면 점 몇 개만 드래그하듯 수정하면 모양이 맞춰진다.
OUTLINE_RIGHT = [
    (27, 250), (43, 247), (57, 242), (62, 239), (65, 236), (68, 233),
    (80, 195), (80, 180), (78, 160), (75, 145), (73, 130), (72, 115),
    (72, 100), (70, 85), (68, 70),  (66, 60),  (64, 50),  (62, 42),
    (60, 35),  (58, 30),  (55, 28), (50, 26), (45, 26), (40, 28),
    (36, 32),  (34, 40),  (32, 50), (30, 65),  (28, 80),  (27, 95),
    (27, 110), (27, 125), (28, 140), (30, 155), (32, 170), (33, 185),
    (34, 200), (32, 212), (30, 225), (28, 238), (26, 250), (27, 250)
]

# ===== 내부 함수들 =====
def parse_line(line: str):
    """ 'time_ms,ch0,...,ch15' → (t:int, [16values]) """
    parts = line.strip().split(',')
    if len(parts) != 17:
        return None, None
    try:
        t = int(parts[0])
        vals = list(map(int, parts[1:]))
        return t, vals
    except:
        return None, None

def get_fake_vals():
    return int(time.time()*1000), [random.randint(500, 3500) for _ in range(16)]

def mirror_left(coords):
    """오른발 좌표를 왼발로 좌우 반전"""
    # 가로 범위를 0~100으로 가정 → x' = 100 - x
    return [(100 - x, y) for (x, y) in coords]

# ===== 메인 =====
def main():
    # 좌/우 발 선택 반영
    coords = SENSOR_COORDS_RIGHT if FOOT == "right" else mirror_left(SENSOR_COORDS_RIGHT)
    outline = OUTLINE_RIGHT if FOOT == "right" else mirror_left(OUTLINE_RIGHT)

    # 시리얼 오픈 (실패하면 FAKE 모드)
    use_fake = False
    ser = None
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"Connected to {PORT}")
    except Exception as e:
        print(f"Serial open failed: {e} → FAKE mode")
        use_fake = True

    # Figure
    plt.ion()
    fig, ax = plt.subplots(figsize=(4.2, 8.0))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 260)
    ax.set_aspect('equal')
    ax.set_title(f"Insole Pressure ({FOOT} foot)")
    ax.set_xticks([]); ax.set_yticks([])

    # 외곽선 그리기
    ox, oy = zip(*outline)
    ax.plot(ox, oy, 'k-', linewidth=1.8, alpha=0.8)

    # 센서 점 초기화
    xs = [p[0] for p in coords]
    ys = [p[1] for p in coords]
    vals = [0]*16
    scat = ax.scatter(xs, ys, c=vals, cmap='hot', s=550, vmin=0, vmax=4095, edgecolors='k', linewidths=0.5)
    cbar = plt.colorbar(scat, ax=ax, pad=0.02)
    cbar.set_label("ADC")

    # --- 헤더 스킵 + 첫 데이터 확보 ---
    first_data = None
    if not use_fake:
        start = time.time()
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                # 3초 이상 아무것도 안 오면 FAKE로 전환
                if time.time() - start > 3.0:
                    print("No serial data → FAKE mode")
                    use_fake = True
                    break
                continue
            if line.startswith("time_ms"):
                print("Header detected, skip")
                break
            else:
                # 숫자 줄이면 바로 첫 데이터로 사용
                t, v = parse_line(line)
                if v is not None:
                    first_data = (t, v)
                    print("No header found, using first data line")
                    break

    # --- 표시 루프 ---
    try:
        if first_data:
            t, vals = first_data
            scat.set_array(np.array(vals))
            ax.set_title(f"Insole Pressure ({FOOT})  t={t} ms")
            plt.pause(0.05)

        while True:
            if use_fake:
                t, vals = get_fake_vals()
            else:
                line = ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                t, vals = parse_line(line)
                if vals is None:
                    continue

            scat.set_array(np.array(vals))
            ax.set_title(f"Insole Pressure ({FOOT})  t={t} ms")
            plt.pause(0.04)
    except KeyboardInterrupt:
        pass
    finally:
        if ser:
            ser.close()
        print("Closed")

if __name__ == "__main__":
    main()
