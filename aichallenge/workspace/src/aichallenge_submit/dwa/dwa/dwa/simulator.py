import math

class Simulator_DWA_robot:
    def __init__(self):
        # 加速度制限
        self.max_accelation = 1.0
        self.max_ang_accelation = 100 * math.pi / 180
        # 速度制限
        self.lim_max_velo = 1.6  # m/s
        self.lim_min_velo = 0.0  # m/s
        self.lim_max_ang_velo = math.pi
        self.lim_min_ang_velo = -math.pi

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