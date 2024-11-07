#include "robot.hpp"
#include <iostream>

Robot::Robot(const Parameters &params)
    : x_(params.ROBOT_INIT_X), y_(params.ROBOT_INIT_Y), th_(params.ROBOT_INIT_TH), u_v_(0.0), u_th_(0.0)
{
  std::cout << params.ROBOT_INIT_X << " " << params.ROBOT_INIT_Y << " " << params.ROBOT_INIT_TH << std::endl;
  traj_x_.push_back(params.ROBOT_INIT_X);
  traj_y_.push_back(params.ROBOT_INIT_Y);
  traj_th_.push_back(params.ROBOT_INIT_TH);
  traj_u_v_.push_back(0.0);
  traj_u_th_.push_back(0.0);
}

double Robot::getX() const
{
  return x_;
}

double Robot::getY() const
{
  return y_;
}

double Robot::getTh() const
{
  return th_;
}

double Robot::getUV() const
{
  return u_v_;
}

double Robot::getUTh() const
{
  return u_th_;
}

void Robot::setX(double x)
{
  x_ = x;
  traj_x_.push_back(x);
}

void Robot::setY(double y)
{
  y_ = y;
  traj_y_.push_back(y);
}

void Robot::setTh(double th)
{
  th_ = th;
  traj_th_.push_back(th);
}

void Robot::setUV(double u_v)
{
  u_v_ = u_v;
  traj_u_v_.push_back(u_v);
}

void Robot::setUTh(double u_th)
{
  u_th_ = u_th;
  traj_u_th_.push_back(u_th);
}

const std::vector<double> &Robot::getTrajX() const
{
  return traj_x_;
}

const std::vector<double> &Robot::getTrajY() const
{
  return traj_y_;
}

const std::vector<double> &Robot::getTrajTh() const
{
  return traj_th_;
}

const std::vector<double> &Robot::getTrajUV() const
{
  return traj_u_v_;
}

const std::vector<double> &Robot::getTrajUTh() const
{
  return traj_u_th_;
}