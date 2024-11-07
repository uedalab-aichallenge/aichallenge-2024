#include "controller.hpp"
#include "dwa.hpp"
#include <cmath>
#include <iostream>
#include <tuple>
#include <rclcpp/rclcpp.hpp>
Controller::Controller(const Parameters &params)
    : params_(params),
      robot_(params),
      dwa_(params),
      iteration_(0) {}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>,
           std::vector<std::vector<Path>>, std::vector<Path>,
           std::vector<double>, std::vector<double>, Course>

Controller::runStep(const std::vector<Obstacle> &obstacles)
{
  auto input = dwa_.calcInput(robot_, obstacles);

  std::vector<Path> valid_paths = input.first;
  Path opt_path = input.second;

  // 最適パスから速度指令を設定
  if (!valid_paths.empty() && opt_path.getX().size() >= 2)
  {
    double dx = opt_path.getX()[opt_path.getX().size() - 1] - opt_path.getX()[opt_path.getX().size() - 2];
    double dy = opt_path.getY()[opt_path.getY().size() - 1] - opt_path.getY()[opt_path.getY().size() - 2];
    double dth = opt_path.getTh()[opt_path.getTh().size() - 1] - opt_path.getTh()[opt_path.getTh().size() - 2];
    double dt = params_.DT;

    // 速度と角速度を計算
    double new_u_v = std::hypot(dx, dy) / dt;
    double new_u_th = dth / dt;

    // 状態を更新
    robot_.setUV(new_u_v);
    robot_.setUTh(new_u_th);
    robot_.setX(opt_path.getX().back());
    robot_.setY(opt_path.getY().back());
    robot_.setTh(opt_path.getTh().back());
  }
  iteration_++;

  // トラジェクトリ情報の取得
  std::vector<double> traj_x = robot_.getTrajX();
  std::vector<double> traj_y = robot_.getTrajY();
  std::vector<double> traj_th = robot_.getTrajTh();
  std::vector<std::vector<Path>> traj_paths = dwa_.getTrajPaths();
  std::vector<Path> traj_opt = dwa_.getTrajOpt();
  std::vector<double> traj_g_x = dwa_.getTrajGX();
  std::vector<double> traj_g_y = dwa_.getTrajGY();
  Course course = dwa_.getCourse();
  return std::make_tuple(traj_x, traj_y, traj_th, traj_paths, traj_opt, traj_g_x, traj_g_y, course);
}

Robot &Controller::getRobot()
{
  return robot_;
}