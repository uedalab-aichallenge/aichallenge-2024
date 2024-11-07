#pragma once

#include <string>

struct Parameters
{
  double ROBOT_INIT_X;
  double ROBOT_INIT_Y;
  double ROBOT_INIT_TH;

  double DT;
  double MAX_SPEED;
  double MIN_SPEED;
  double MAX_YAWRATE;
  double MIN_YAWRATE;
  double MAX_ACCEL;
  double MAX_DYAWRATE;
  double V_RESOLUTION;
  double YAWRATE_RESOLUTION;

  double PREDICT_TIME;
  double ROBOT_RADIUS;

  double WEIGHT_ANGLE;
  double WEIGHT_VELOCITY;
  double WEIGHT_OBSTACLE;
  double WEIGHT_DISTANCE;

  std::string LEFT_LANE_BOUND_FILE;
  std::string RIGHT_LANE_BOUND_FILE;
  std::string CENTER_LANE_LINE_FILE;

  double LOOKAHEAD_DISTANCE;

  double X_OFFSET;
  double Y_OFFSET;

  double OBS_SIZE;
  double STEERING_TIRE_ANGLE_GAIN; // New parameter

};
