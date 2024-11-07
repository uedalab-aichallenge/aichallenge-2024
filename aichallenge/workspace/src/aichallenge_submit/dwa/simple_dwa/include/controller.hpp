#pragma once
#include "robot.hpp"
#include "dwa.hpp"
#include <vector>
#include "obstacle.hpp"
#include "types.hpp" // Parameters構造体をインクルード

class Controller
{
public:
  Controller(const Parameters &params); // パラメータを受け取るコンストラクタ

  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>,
             std::vector<std::vector<Path>>, std::vector<Path>,
             std::vector<double>, std::vector<double>, Course>
  runStep(const std::vector<Obstacle> &obstacles);

  Robot &getRobot();

private:
  Robot robot_;
  DWA dwa_;
  int iteration_;
  Parameters params_;
};
