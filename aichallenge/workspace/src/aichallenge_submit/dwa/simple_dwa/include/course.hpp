#pragma once

#include <vector>
#include <string>
#include "obstacle.hpp"
#include "types.hpp"

class Course
{
public:
  Course(const Parameters &params);

  std::pair<double, double> getNearestPoint(double x, double y) const;
  std::pair<double, double> getNextTargetPoint(double x, double y, double th, double lookahead_distance = 2.0);

private:
  Parameters params_;
  std::vector<std::pair<double, double>> left_lane_;
  std::vector<std::pair<double, double>> right_lane_;
  std::vector<std::pair<double, double>> center_lane_;

  void loadCSV(const std::string &file_path, std::vector<std::pair<double, double>> &lane);
  void applyOffset(std::vector<std::pair<double, double>> &lane);
  int findNearestIndex(double x, double y) const;
};
