#pragma once

#include <vector>
#include "types.hpp"

class Simulator
{
public:
  Simulator();
  Simulator(const Parameters &params);

  std::vector<double> predictState(double ang_velo, double velo, double x, double y, double th, double dt, int pre_step);
private:
  Parameters params_;
};
