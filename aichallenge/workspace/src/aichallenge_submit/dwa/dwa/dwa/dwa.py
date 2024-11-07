import math
import numpy as np
from utils import min_max_normalize, angle_range_corrector
from obstacle import Obstacle
from config import *
from course import Course
import sys
from config import LOOKAHEAD_DISTANCE

class Simulator_DWA_robot:
    def __init__(self):
        self.max_accelation = MAX_ACCEL
        self.max_ang_accelation = MAX_DYAWRATE
        self.lim_max_velo = MAX_SPEED
        self.lim_min_velo = MIN_SPEED
        self.lim_max_ang_velo = MAX_YAWRATE
        self.lim_min_ang_velo = MIN_YAWRATE

    def predict_state(self, ang_velo, velo, x, y, th, dt, pre_step):
        next_xs = []
        next_ys = []
        next_ths = []

        for _ in range(pre_step):
            temp_x = velo * math.cos(th) * dt + x
            temp_y = velo * math.sin(th) * dt + y
            temp_th = ang_velo * dt + th

            next_xs.append(temp_x)
            next_ys.append(temp_y)
            next_ths.append(temp_th)

            x = temp_x
            y = temp_y
            th = temp_th

        return next_xs, next_ys, next_ths

class Path:
    def __init__(self, u_th, u_v):
        self.x = None
        self.y = None
        self.th = None
        self.u_v = u_v
        self.u_th = u_th


class DWA():
    def __init__(self):
        self.simu_robot = Simulator_DWA_robot()
        self.pre_time = PREDICT_TIME
        self.pre_step = int(self.pre_time / DT)
        self.delta_velo = V_RESOLUTION
        self.delta_ang_velo = YAWRATE_RESOLUTION
        self.samplingtime = DT
        self.weight_angle = WEIGHT_ANGLE
        self.weight_velo = WEIGHT_VELOCITY
        self.weight_obs = WEIGHT_OBSTACLE
        self.weight_distance = WEIGHT_DISTANCE
        try:
            self.course = Course(LEFT_LANE_BOUND_FILE, RIGHT_LANE_BOUND_FILE, CENTER_LANE_LINE_FILE)
        except FileNotFoundError as e:
            print("CSVファイルのパスが正しいか確認してください。")
            sys.exit(1)

        # すべてのPathを保存
        self.traj_paths = []
        self.traj_opt = []

    def calc_input(self, robot, obstacles):
        # Path作成
        paths = self._make_path(robot)
        print(f"Number of paths generated: {len(paths)}")

        # Path評価
        g_x, g_y = self.course.get_next_target_point(robot.x, robot.y, robot.th, LOOKAHEAD_DISTANCE)

        opt_path, valid_paths = self._eval_path(paths, g_x, g_y, robot, obstacles)

        self.traj_opt.append(opt_path)
        if not hasattr(self, 'traj_g_x'):
            self.traj_g_x = []
            self.traj_g_y = []
        self.traj_g_x.append(g_x)
        self.traj_g_y.append(g_y)

        self.traj_paths.append(valid_paths)  # 有効なパスのみを保存

        return valid_paths, opt_path

    def _make_path(self, state):
        # 角度と速度の範囲算出
        min_ang_velo, max_ang_velo, min_velo, max_velo = self._calc_range_velos(state)
        max_velo = MAX_SPEED
        # 全てのpathのリスト
        paths = []

        # 角速度と速度の組み合わせを全探索
        for ang_velo in np.arange(min_ang_velo, max_ang_velo, self.delta_ang_velo):
            for velo in np.arange(min_velo, max_velo, self.delta_velo):

                path = Path(ang_velo, velo)

                next_x, next_y, next_th \
                    = self.simu_robot.predict_state(ang_velo, velo, state.x, state.y, state.th, self.samplingtime, self.pre_step)

                path.x = next_x
                path.y = next_y
                path.th = next_th

                # 作ったpathを追加
                paths.append(path)

        # 時刻歴Pathを保存
        self.traj_paths.append(paths)

        return paths

    def _calc_range_velos(self, state): # 角速度と角度の範囲決定①
        # 角速度
        range_ang_velo = self.samplingtime * self.simu_robot.max_ang_accelation
        min_ang_velo = state.u_th - range_ang_velo
        max_ang_velo = state.u_th + range_ang_velo
        # 最小値
        if min_ang_velo < self.simu_robot.lim_min_ang_velo:
            min_ang_velo = self.simu_robot.lim_min_ang_velo
        # 最大値
        if max_ang_velo > self.simu_robot.lim_max_ang_velo:
            max_ang_velo = self.simu_robot.lim_max_ang_velo

        # 速度
        range_velo = self.samplingtime * self.simu_robot.max_accelation
        min_velo = state.u_v - range_velo
        max_velo = state.u_v + range_velo
        # 最小値
        if min_velo < self.simu_robot.lim_min_velo:
            min_velo = self.simu_robot.lim_min_velo
        # 最大値
        if max_velo > self.simu_robot.lim_max_velo:
            max_velo = self.simu_robot.lim_max_velo

        return min_ang_velo, max_ang_velo, min_velo, max_velo

    def _eval_path(self, paths, g_x, g_y, state, obstacles):
        # 一番近い障害物判定
        nearest_obs = self._calc_nearest_obs(state, obstacles)
        valid_paths = []
        score_heading_angles = []
        score_heading_velos = []
        score_obstacles = []

        # 全てのpathで評価を検索
        for path in paths:
            # (1) heading_angle
            angle_score = self._heading_angle(path, g_x, g_y)
            # (2) heading_velo
            velo_score = self._heading_velo(path)
            # (3) obstacle
            obs_score = self._obstacle(path, nearest_obs)
            # パスが有効かチェック（スコアにinfやnanが含まれていないか）
            if np.isfinite(angle_score) and np.isfinite(velo_score) and np.isfinite(obs_score):
                score_heading_angles.append(angle_score)
                score_heading_velos.append(velo_score)
                score_obstacles.append(obs_score)
                valid_paths.append(path)
            else:
                # 無効なパスはスキップ
                continue

        if not valid_paths:
            print("No valid paths found. All paths are either out of bounds or colliding with obstacles.")
            raise ValueError("有効なPathが存在しません。全てのPathがコース外か障害物と衝突しています。")

        # スコアの正規化
        score_heading_angles = min_max_normalize(score_heading_angles)
        score_heading_velos = min_max_normalize(score_heading_velos)
        score_obstacles = min_max_normalize(score_obstacles)

        # ゴールへの距離を計算
        distances_to_goal = []
        for path in valid_paths:
            last_x = path.x[-1]
            last_y = path.y[-1]
            distance = math.hypot(g_x - last_x, g_y - last_y)
            distances_to_goal.append(distance)

        # 距離を反転（近いほど良い）して正規化
        inverted_distances = [-d for d in distances_to_goal]
        distances_normalized = min_max_normalize(inverted_distances)

        # 最適なpathを探索
        score = -float('inf')  # スコアの初期値を負の無限大に設定
        opt_path = None        # opt_path を初期化
        opt_cost_components = {}

        for k in range(len(valid_paths)):
            temp_score = (self.weight_angle * score_heading_angles[k] +
                          self.weight_velo * score_heading_velos[k] +
                          self.weight_obs * score_obstacles[k] +
                          self.weight_distance * distances_normalized[k])  # 距離スコアを加算

            if temp_score > score:
                opt_path = valid_paths[k]
                score = temp_score
                opt_cost_components = {
                    'angle_score': score_heading_angles[k],
                    'velo_score': score_heading_velos[k],
                    'obs_score': score_obstacles[k],
                    'distance_score': distances_normalized[k]
                }

        if opt_path is None:
            # フォールバック: 有効なパスがあれば最初を選択
            if valid_paths:
                opt_path = valid_paths[0]
            else:
                raise ValueError("Pathが生成されていません。")

        # 最適パスのコスト情報を出力
        print("選択されたパスのコスト:")
        print(f"  角度スコア: {opt_cost_components.get('angle_score', 'N/A')}")
        print(f"  速度スコア: {opt_cost_components.get('velo_score', 'N/A')}")
        print(f"  障害物スコア: {opt_cost_components.get('obs_score', 'N/A')}")
        print(f"  ゴール距離スコア: {opt_cost_components.get('distance_score', 'N/A')}")

        return opt_path, valid_paths  # 有効なパスも返すように変更

    def _heading_angle(self, path, g_x, g_y): # ゴールに向いているか
        # 終端の向き
        last_x = path.x[-1]
        last_y = path.y[-1]
        last_th = path.th[-1]

        # 角度計算
        angle_to_goal = math.atan2(g_y-last_y, g_x-last_x)

        # score計算
        score_angle = angle_to_goal - last_th

        # ぐるぐる防止
        score_angle = abs(angle_range_corrector(score_angle))

        # 最大と最小をひっくり返す
        score_angle = math.pi - score_angle

        return score_angle

    def _heading_velo(self, path): # 速くんでいるか（直進）

        score_heading_velo = path.u_v

        return score_heading_velo

    def _calc_nearest_obs(self, state, obstacles):
        area_dis_to_obs = 10 # パラメー（何メートル考慮するか，本当は制動距離）
        nearest_obs = [] # あるエリアに入ってる障害物
        for obs in obstacles:
            temp_dis_to_obs = math.sqrt((state.x - obs.x) ** 2 + (state.y - obs.y) ** 2)

            if temp_dis_to_obs < area_dis_to_obs :
                nearest_obs.append(obs)

        return nearest_obs

    def _obstacle(self, path, nearest_obs):
        # 障害物回避（エリアに入ったらその線は使わない）/ (障害物ともっとも近い距離)))
        score_obstacle = 2.1
        temp_dis_to_obs = 0.0

        for i in range(len(path.x)):
            for obs in nearest_obs:
                temp_dis_to_obs = math.sqrt((path.x[i] - obs.x) ** 2 + (path.y[i] - obs.y) ** 2)

                if temp_dis_to_obs < obs.size + 1.0:
                    return -float('inf')  # 無効なパスを示すスコアを返す

                if temp_dis_to_obs < score_obstacle:
                    score_obstacle = temp_dis_to_obs  # 一番近いところ

        return score_obstacle









