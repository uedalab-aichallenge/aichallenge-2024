import numpy as np
import os
from config import X_OFFSET, Y_OFFSET


class Course:
    def __init__(self, left_lane_file, right_lane_file, center_lane_file):
        self.left_lane = self._load_csv(left_lane_file)
        self.right_lane = self._load_csv(right_lane_file)
        self.center_lane = self._load_csv(center_lane_file)

    def _load_csv(self, file_path):
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"ファイルが見つかりません: {file_path}")
        data = np.loadtxt(file_path, delimiter=',', skiprows=1)
        return self._apply_offset(data)

    def _apply_offset(self, data):
        data[:, 0] -= X_OFFSET
        data[:, 1] -= Y_OFFSET
        return data

    def get_nearest_point(self, x, y):
        distances = np.sqrt(np.sum((self.center_lane[:, :2] - np.array([x, y]))**2, axis=1))
        nearest_index = np.argmin(distances)
        return self.center_lane[nearest_index, :2]

    def is_within_bounds(self, x, y):
        point = np.array([x, y])
        left_distances = np.cross(self.left_lane[1:, :2] - self.left_lane[:-1, :2], point - self.left_lane[:-1, :2])
        right_distances = np.cross(self.right_lane[1:, :2] - self.right_lane[:-1, :2], point - self.right_lane[:-1, :2])
        return np.all(left_distances >= 0) and np.all(right_distances <= 0)

    def get_next_target_point(self, x, y, th, lookahead_distance=2.0):
        nearest_index = self._find_nearest_index(x, y)
        
        for i in range(nearest_index, len(self.center_lane)):
            point = self.center_lane[i]
            distance = np.sqrt((point[0] - x)**2 + (point[1] - y)**2)
            
            # if distance >= lookahead_distance:
            #     angle_to_point = np.arctan2(point[1] - y, point[0] - x)
            #     angle_diff = abs(angle_to_point - th)
            #     if angle_diff <= np.pi/2 or angle_diff >= 3*np.pi/2:
            #         return point[0], point[1]

            if distance >= lookahead_distance:
                angle_to_point = np.arctan2(point[1] - y, point[0] - x)
                angle_diff = abs(angle_to_point - th)
                return point[0], point[1]

        return self.center_lane[-1][0], self.center_lane[-1][1]

    def _find_nearest_index(self, x, y):
        distances = np.sqrt(np.sum((self.center_lane[:, :2] - np.array([x, y]))**2, axis=1))
        return np.argmin(distances)

    def is_path_within_bounds(self, path_x, path_y):
        for i, (x, y) in enumerate(zip(path_x, path_y)):
            if not self.is_within_bounds(x, y):
                print(f"Path point {i} out of bounds: ({x}, {y})")
                return False
        return True
