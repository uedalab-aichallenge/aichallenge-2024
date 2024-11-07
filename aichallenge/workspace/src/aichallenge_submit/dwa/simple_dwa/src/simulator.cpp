#include "simulator.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
Simulator::Simulator() {}

Simulator::Simulator(const Parameters &params)
    : params_(params) {}

std::vector<double> Simulator::predictState(double ang_velo, double velo, double x, double y, double th, double dt, int pre_step)
{
  std::vector<double> next_xs;
  std::vector<double> next_ys;
  std::vector<double> next_ths;
  for (int i = 0; i < pre_step; ++i)
  {
    double temp_x = velo * std::cos(th) * dt + x;
    double temp_y = velo * std::sin(th) * dt + y;
    double temp_th = ang_velo * dt + th;

    next_xs.push_back(temp_x);
    next_ys.push_back(temp_y);
    next_ths.push_back(temp_th);

    x = temp_x;
    y = temp_y;
    th = temp_th;
  }

  std::vector<double> result;
  result.insert(result.end(), next_xs.begin(), next_xs.end());
  result.insert(result.end(), next_ys.begin(), next_ys.end());
  result.insert(result.end(), next_ths.begin(), next_ths.end());
  return result;
}