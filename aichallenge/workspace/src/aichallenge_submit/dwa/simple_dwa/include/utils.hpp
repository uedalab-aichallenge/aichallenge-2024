#pragma once
#include <vector>
#include <stdexcept>
#include "obstacle.hpp"
#include "types.hpp"

class Obstacle; // 前方宣言

std::vector<double> minMaxNormalize(const std::vector<double> &data);
double angleRangeCorrector(double angle);
std::vector<Obstacle> loadObstacles(const std::vector<std::string> &csv_files, const Parameters &params);