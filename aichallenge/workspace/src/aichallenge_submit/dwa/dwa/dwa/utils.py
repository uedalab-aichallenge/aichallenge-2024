import math
import numpy as np
import csv
from config import LEFT_LANE_BOUND_FILE, RIGHT_LANE_BOUND_FILE, X_OFFSET, Y_OFFSET
from obstacle import Obstacle

def min_max_normalize(data):
    data = np.array(data, dtype=np.float64)

    # Check if data contains inf or NaN
    if not np.all(np.isfinite(data)):
        raise ValueError("データに無限大またはNaNが含まれています。")

    max_data = np.max(data)
    min_data = np.min(data)

    if max_data - min_data == 0:
        normalized_data = np.zeros_like(data)
    else:
        normalized_data = (data - min_data) / (max_data - min_data)

    return normalized_data

# 角度補正用
def angle_range_corrector(angle):
    if angle > math.pi:
        while angle > math.pi:
            angle -=  2 * math.pi
    elif angle < -math.pi:
        while angle < -math.pi:
            angle += 2 * math.pi

    return angle

def load_obstacles():
    obstacles = []
    csv_files = [LEFT_LANE_BOUND_FILE, RIGHT_LANE_BOUND_FILE]

    for file_path in csv_files:
        try:
            with open(file_path, newline='') as csvfile:
                reader = csv.reader(csvfile)
                next(reader, None)  # ヘッダーをスキップ
                for row in reader:
                    if len(row) < 2:
                        continue  # 不正な行をスキップ
                    try:
                        x, y = map(float, row[:2])
                        # Apply offset to obstacle coordinates
                        x -= X_OFFSET
                        y -= Y_OFFSET
                        obstacles.append(Obstacle(x, y, size=0.2))  # サイズは必要に応じて調整
                    except ValueError:
                        print(f"無効なデータ行: {row}")
                        continue
        except FileNotFoundError:
            print(f"ファイルが見つかりません: {file_path}")
            continue

    return obstacles
