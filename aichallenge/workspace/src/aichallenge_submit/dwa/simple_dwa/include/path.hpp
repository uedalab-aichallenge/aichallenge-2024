#pragma once

#include <vector>

class Path
{
public:
  Path(double u_th, double u_v);

  void setX(const std::vector<double> &x);
  void setY(const std::vector<double> &y);
  void setTh(const std::vector<double> &th);

  const std::vector<double> &getX() const;
  const std::vector<double> &getY() const;
  const std::vector<double> &getTh() const;
  double getUV() const;
  double getUTh() const;

private:
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> th_;
  double u_v_;
  double u_th_;
};
