import numpy as np
import math
import time
from robot import Robot
from dwa import DWA
from config import DT

class Main_controller:
    def __init__(self):
        self.robot = Robot()
        self.controller = DWA()
        self.samplingtime = DT
        self.iteration = 0

    def run_step(self, obstacles):
        start_time = time.time()
        # Input calculation
        valid_paths, opt_path = self.controller.calc_input(self.robot, obstacles)

        # 最適パスから速度指令を設定
        if opt_path and len(opt_path.x) >= 2:
            # 最適パスの最後の位置と前の位置から速度を計算
            dx = opt_path.x[-1] - opt_path.x[-2]
            dy = opt_path.y[-1] - opt_path.y[-2]
            dth = opt_path.th[-1] - opt_path.th[-2]
            dt = self.samplingtime

            # 速度と角速度を計算
            self.robot.u_v = math.hypot(dx, dy) / dt
            self.robot.u_th = dth / dt

            # 状態を更新
            self.robot.x = opt_path.x[-1]
            self.robot.y = opt_path.y[-1]
            self.robot.th = opt_path.th[-1]

        self.iteration += 1
        end_time = time.time()
        print(f"Iteration: {self.iteration}, Time per step: {end_time - start_time:.4f} sec")

        return (
            self.robot.traj_x,
            self.robot.traj_y,
            self.robot.traj_th,
            self.controller.traj_paths,
            self.controller.traj_opt,
            self.controller.traj_g_x,
            self.controller.traj_g_y,
            self.controller.course
        )