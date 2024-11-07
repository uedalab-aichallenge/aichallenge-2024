#pragma once

#include <vector>
#include "types.hpp"
class Robot
{
public:
  Robot(const Parameters &params);

  double getX() const;
  double getY() const;
  double getTh() const;
  double getUV() const;
  double getUTh() const;

  void setX(double x);
  void setY(double y);
  void setTh(double th);
  void setUV(double u_v);
  void setUTh(double u_th);

  const std::vector<double> &getTrajX() const;
  const std::vector<double> &getTrajY() const;
  const std::vector<double> &getTrajTh() const;
  const std::vector<double> &getTrajUV() const;
  const std::vector<double> &getTrajUTh() const;

private:
  Parameters params_;
  double x_;
  double y_;
  double th_;
  double u_v_;
  double u_th_;

  std::vector<double> traj_x_;
  std::vector<double> traj_y_;
  std::vector<double> traj_th_;
  std::vector<double> traj_u_v_;
  std::vector<double> traj_u_th_;
};
