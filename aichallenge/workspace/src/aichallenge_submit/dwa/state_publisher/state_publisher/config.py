import math

# Robot parameters
ROBOT_INIT_X = 5.0
ROBOT_INIT_Y = -5.0
ROBOT_INIT_TH = 2.7

# DWA parameters
MAX_SPEED = 5.55556  # m/s(20km/h)
MIN_SPEED = 0.0
MAX_YAWRATE = math.pi
MIN_YAWRATE = -math.pi
MAX_ACCEL = 1.0
MAX_DYAWRATE = 300.0 * math.pi / 180.0
V_RESOLUTION = 0.5
YAWRATE_RESOLUTION = 0.04

# Simulator parameters
DT = 0.1  # Main_controllerとdwaで共通のsamplingtime
PREDICT_TIME = 2.0
ROBOT_RADIUS = 0.2

# Cost parameters
WEIGHT_ANGLE = 0.04
WEIGHT_VELOCITY = 1.2
WEIGHT_OBSTACLE = 1000.0
WEIGHT_DISTANCE = 1.0  # ゴールへの距離の重み

# Course parameters
LEFT_LANE_BOUND_FILE = 'csv_files/outer_track_interpolated.csv'
RIGHT_LANE_BOUND_FILE = 'csv_files/inner_track_interpolated.csv'
CENTER_LANE_LINE_FILE = 'csv_files/center_lane_line.csv'

# DWA parameters
LOOKAHEAD_DISTANCE = 9.0

X_OFFSET = 89633.15625
Y_OFFSET = 43127.796875