#pragma once
#include <vector>
#include "robot.hpp"
#include "course.hpp"
#include "path.hpp"
#include "obstacle.hpp"
#include "simulator.hpp"
#include "types.hpp" // Parameters構造体をインクルード

class DWA
{
public:
  DWA(const Parameters& params);

  std::pair<std::vector<Path>, Path> calcInput(class Robot &robot, const std::vector<Obstacle> &obstacles);

  const std::vector<std::vector<Path>>& getTrajPaths() const;
  const std::vector<Path>& getTrajOpt() const;
  const std::vector<double>& getTrajGX() const;
  const std::vector<double>& getTrajGY() const;
  const Course& getCourse() const;

private:
  Simulator simu_robot_;
  double pre_time_;
  int pre_step_;
  Parameters params_;
  Course course_;

  std::vector<std::vector<Path>> traj_paths_;
  std::vector<Path> traj_opt_;
  std::vector<double> traj_g_x_;
  std::vector<double> traj_g_y_;

  std::vector<Path> makePath(class Robot &robot);
  std::pair<Path, std::vector<Path>> evalPath(const std::vector<Path> &paths, double g_x, double g_y, class Robot &robot, const std::vector<Obstacle> &obstacles);

  double headingAngle(const Path &path, double g_x, double g_y) const;
  double headingVelo(const Path &path) const;
  std::vector<Obstacle> calcNearestObs(const class Robot &state, const std::vector<Obstacle> &obstacles) const;
  double obstacleScore(const Path &path, const std::vector<Obstacle> &nearest_obs) const;
  void addTrajectory(Path opt_path, double g_x, double g_y, const std::vector<Path>& valid_paths);
};
