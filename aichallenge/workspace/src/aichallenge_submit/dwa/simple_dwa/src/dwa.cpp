#include "dwa.hpp"
#include "utils.hpp"
#include <iostream>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>

DWA::DWA(const Parameters &params)
    : params_(params),
      simu_robot_(params),
      course_(params) {}

std::pair<std::vector<Path>, Path> DWA::calcInput(Robot &robot, const std::vector<Obstacle> &obstacles)
{
  // Path作成
  std::vector<Path> paths = makePath(robot);
  std::cout << "Number of paths generated: " << paths.size() << std::endl;

  // Path評価
  auto target_point = course_.getNextTargetPoint(robot.getX(), robot.getY(), robot.getTh(), params_.LOOKAHEAD_DISTANCE);
  double g_x = target_point.first;
  double g_y = target_point.second;

  auto eval_result = evalPath(paths, g_x, g_y, robot, obstacles);
  Path opt_path = eval_result.first;
  std::vector<Path> valid_paths = eval_result.second;

  // サイズ管理関数を呼び出す
  addTrajectory(opt_path, g_x, g_y, valid_paths);
  return {valid_paths, opt_path};
}

std::vector<Path> DWA::makePath(Robot &robot)
{
  // 角度と速度の範囲算出
  double min_ang_velo, max_ang_velo, min_velo, max_velo;
  // 角速度の範囲
  double range_ang_velo = params_.DT * params_.MAX_DYAWRATE;
  min_ang_velo = robot.getUTh() - range_ang_velo;
  max_ang_velo = robot.getUTh() + range_ang_velo;

  // 範囲制限
  min_ang_velo = std::max(min_ang_velo, params_.MIN_YAWRATE);
  max_ang_velo = std::min(max_ang_velo, params_.MAX_YAWRATE);
  // 速度の範囲
  double range_velo = params_.DT * params_.MAX_ACCEL;
  min_velo = robot.getUV() - range_velo;
  max_velo = robot.getUV() + range_velo;
  min_velo = std::max(min_velo, params_.MIN_SPEED);
  max_velo = std::min(max_velo, params_.MAX_SPEED);
  // 直線部分のために速度範囲を広げる
  if (robot.getUV() > params_.MIN_SPEED) {
      min_velo = std::max(min_velo, params_.MIN_SPEED);
      max_velo = std::min(max_velo, params_.MAX_SPEED);
  }

  if (min_ang_velo > max_ang_velo)
  {
    min_ang_velo = max_ang_velo - params_.YAWRATE_RESOLUTION;
  }
  if (min_velo > max_velo)
  {
    min_velo = max_velo - params_.V_RESOLUTION;
  }

  // 全てのpathのリスト
  std::vector<Path> paths;
  int count = 0;
  for (double ang_velo = min_ang_velo; ang_velo < max_ang_velo; ang_velo += params_.YAWRATE_RESOLUTION)
  {
    for (double velo = min_velo; velo < max_velo; velo += params_.V_RESOLUTION)
    {
      Path path(ang_velo, velo);
      pre_step_ = static_cast<int>(params_.PREDICT_TIME / params_.DT);
      std::vector<double> predicted = simu_robot_.predictState(ang_velo, velo, robot.getX(), robot.getY(), robot.getTh(), params_.DT, pre_step_);
      std::vector<double> next_x(predicted.begin(), predicted.begin() + pre_step_);
      std::vector<double> next_y(predicted.begin() + pre_step_, predicted.begin() + 2 * pre_step_);
      std::vector<double> next_th(predicted.begin() + 2 * pre_step_, predicted.end());

      path.setX(next_x);
      path.setY(next_y);
      path.setTh(next_th);

      paths.push_back(path);
    }
    count++;
  }
  return paths;
}

std::pair<Path, std::vector<Path>> DWA::evalPath(const std::vector<Path> &paths, double g_x, double g_y, Robot &robot, const std::vector<Obstacle> &obstacles)
{
  // 一番近い障害物判定
  std::vector<Obstacle> nearest_obs = calcNearestObs(robot, obstacles);
  std::vector<Path> valid_paths;
  std::vector<double> score_heading_angles;
  std::vector<double> score_heading_velos;
  std::vector<double> score_obstacles;

  // 全てのpathで評価を検索
  for (const auto &path : paths)
  {
    double angle_score = headingAngle(path, g_x, g_y);
    double velo_score = headingVelo(path);
    double obs_score = obstacleScore(path, nearest_obs);

    if (std::isfinite(angle_score) && std::isfinite(velo_score) && std::isfinite(obs_score))
    {
      score_heading_angles.push_back(angle_score);
      score_heading_velos.push_back(velo_score);
      score_obstacles.push_back(obs_score);
      valid_paths.push_back(path);
    }
    else
    {
      // 無効なパスはスキップ
      continue;
    }
  }

  if (valid_paths.empty())
  {
    std::cerr << "No valid paths found. All paths are either out of bounds or colliding with obstacles." << std::endl;
    throw std::runtime_error("有効なPathが存在しません。全てのPathがコース外か障害物と衝突しています。");
  }

  // スコアの正規化
  std::vector<double> score_heading_angles_norm = minMaxNormalize(score_heading_angles);
  std::vector<double> score_heading_velos_norm = minMaxNormalize(score_heading_velos);
  std::vector<double> score_obstacles_norm = minMaxNormalize(score_obstacles);

  // ゴールへの距離を計算
  std::vector<double> distances_to_goal;
  for (const auto &path : valid_paths)
  {
    double last_x = path.getX().back();
    double last_y = path.getY().back();
    double distance = std::hypot(g_x - last_x, g_y - last_y);
    distances_to_goal.push_back(distance);
  }

  // 距離を反転（近いほど良い）して正規化
  std::vector<double> inverted_distances;
  for (const auto &d : distances_to_goal)
  {
    inverted_distances.push_back(-d);
  }
  std::vector<double> distances_normalized = minMaxNormalize(inverted_distances);

  // 最適なpathを探索
  double max_score = -std::numeric_limits<double>::infinity();
  int opt_index = -1;
  std::vector<double> opt_cost_components(4, 0.0); // 角度, 速度, 障害物, 距離

  for (size_t k = 0; k < valid_paths.size(); ++k)
  {
    double temp_score = (params_.WEIGHT_ANGLE * score_heading_angles_norm[k] +
                         params_.WEIGHT_VELOCITY * score_heading_velos_norm[k] +
                         params_.WEIGHT_OBSTACLE * score_obstacles_norm[k] +
                         params_.WEIGHT_DISTANCE * distances_normalized[k]);
    if (temp_score > max_score)
    {
      max_score = temp_score;
      opt_index = static_cast<int>(k);
      opt_cost_components[0] = score_heading_angles_norm[k];
      opt_cost_components[1] = score_heading_velos_norm[k];
      opt_cost_components[2] = score_obstacles_norm[k];
      opt_cost_components[3] = distances_normalized[k];
    }
  }

  Path opt_path = valid_paths[opt_index];

  // 最適パスのコスト情報を出力
  std::cout << "選択されたパスのコスト:" << std::endl;
  std::cout << "  角度スコア: " << opt_cost_components[0] << std::endl;
  std::cout << "  速度スコア: " << opt_cost_components[1] << std::endl;
  std::cout << "  障害物スコア: " << opt_cost_components[2] << std::endl;
  std::cout << "  ゴール距離スコア: " << opt_cost_components[3] << std::endl;

  return {opt_path, valid_paths};
}

double DWA::headingAngle(const Path &path, double g_x, double g_y) const
{
  double last_x = path.getX().back();
  double last_y = path.getY().back();
  double last_th = path.getTh().back();

  // double angle_to_goal = std::atan2(g_y - last_y, g_x - last_x);
  // double score_angle = angle_to_goal - last_th;
  // score_angle = std::abs(angleRangeCorrector(score_angle));
  // score_angle = M_PI - score_angle;

  // return score_angle;

  // 目標への角度を計算
  double angle_to_goal = std::atan2(g_y - last_y, g_x - last_x);
  
  // 現在の向きとの角度差を計算
  double angle_diff = angle_to_goal - last_th;

  // 角度を正規化
  angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff)); // -πからπの範囲に正規化

  // スコアを計算（角度差が小さいほど良い）
  double score_angle = 1.0 - std::abs(angle_diff) / M_PI; // 0から1の範囲にスケーリング

  return score_angle;
}

double DWA::headingVelo(const Path &path) const
{
  // return path.getUV();

  double velocity = path.getUV();
  double max_velocity = params_.MAX_SPEED; // 最大速度を取得

  // スコアを計算（速度が高いほど良い）
  double score_velocity = velocity / max_velocity; // 0から1の範囲にスケーリング

  // スコアが1を超えないように制限
  return std::min(score_velocity, 1.0);
}

std::vector<Obstacle> DWA::calcNearestObs(const Robot &state, const std::vector<Obstacle> &obstacles) const
{
  double area_dis_to_obs = 30.0; // パラメーター
  std::vector<Obstacle> nearest_obs;
  for (const auto &obs : obstacles)
  {
    double temp_dis = std::hypot(state.getX() - obs.getX(), state.getY() - obs.getY());
    if (temp_dis < area_dis_to_obs)
    {
      nearest_obs.push_back(obs);
    }
  }
  return nearest_obs;
}

double DWA::obstacleScore(const Path &path, const std::vector<Obstacle> &nearest_obs) const
{
  double score_obstacle = 2.1;
  for (size_t i = 0; i < path.getX().size(); ++i)
  {
    for (const auto &obs : nearest_obs)
    {
      double temp_dis = std::hypot(path.getX()[i] - obs.getX(), path.getY()[i] - obs.getY());
      if (temp_dis < obs.getSize())
      {
        return -std::numeric_limits<double>::infinity(); // 無効なパス
      }
      if (temp_dis < score_obstacle)
      {
        score_obstacle = temp_dis;
      }
    }
  }
  return score_obstacle;
}

const std::vector<std::vector<Path>> &DWA::getTrajPaths() const
{
  return traj_paths_;
}

const std::vector<Path> &DWA::getTrajOpt() const
{
  return traj_opt_;
}

const std::vector<double> &DWA::getTrajGX() const
{
  return traj_g_x_;
}

const std::vector<double> &DWA::getTrajGY() const
{
  return traj_g_y_;
}

const Course &DWA::getCourse() const
{
  return course_;
}

// サイズ管理関数の作成
void DWA::addTrajectory(Path opt_path, double g_x, double g_y, const std::vector<Path> &valid_paths)
{
  const size_t MAX_TRAJ_SIZE = 10;
  if (traj_opt_.size() >= MAX_TRAJ_SIZE)
  {
    traj_opt_.erase(traj_opt_.begin());
  }
  traj_opt_.push_back(opt_path);

  if (traj_g_x_.size() >= MAX_TRAJ_SIZE)
  {
    traj_g_x_.erase(traj_g_x_.begin());
  }
  traj_g_x_.push_back(g_x);

  if (traj_g_y_.size() >= MAX_TRAJ_SIZE)
  {
    traj_g_y_.erase(traj_g_y_.begin());
  }
  traj_g_y_.push_back(g_y);

  if (traj_paths_.size() >= MAX_TRAJ_SIZE)
  {
    traj_paths_.erase(traj_paths_.begin());
  }
  traj_paths_.push_back(valid_paths);
}