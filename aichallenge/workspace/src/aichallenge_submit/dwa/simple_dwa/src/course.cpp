#include "course.hpp"
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

Course::Course(const Parameters &params)
    : params_(params)
{
  loadCSV(params_.LEFT_LANE_BOUND_FILE, left_lane_);
  loadCSV(params_.RIGHT_LANE_BOUND_FILE, right_lane_);
  loadCSV(params_.CENTER_LANE_LINE_FILE, center_lane_);
  applyOffset(left_lane_);
  applyOffset(right_lane_);
  applyOffset(center_lane_);
}

void Course::loadCSV(const std::string &file_path, std::vector<std::pair<double, double>> &lane)
{
  std::ifstream file(file_path);
  if (!file.is_open())
  {
    throw std::runtime_error("ファイルが見つかりません: " + file_path);
  }

  std::string line;
  getline(file, line); // ヘッダーをスキップ
  while (getline(file, line))
  {
    std::stringstream ss(line);
    std::string x_str, y_str;
    if (!getline(ss, x_str, ',') || !getline(ss, y_str, ','))
    {
      continue; // 不正な行をスキップ
    }
    try
    {
      double x = std::stod(x_str);
      double y = std::stod(y_str);
      lane.emplace_back(x, y);
    }
    catch (...)
    {
      // 無効なデータ行はスキップ
      continue;
    }
  }
}

void Course::applyOffset(std::vector<std::pair<double, double>> &lane)
{
  for (auto &point : lane)
  {
    point.first -= params_.X_OFFSET;
    point.second -= params_.Y_OFFSET;
  }
}

std::pair<double, double> Course::getNearestPoint(double x, double y) const
{
  double min_distance = std::numeric_limits<double>::max();
  std::pair<double, double> nearest_point = {0.0, 0.0};
  for (const auto &point : center_lane_)
  {
    double distance = std::hypot(point.first - x, point.second - y);
    if (distance < min_distance)
    {
      min_distance = distance;
      nearest_point = point;
    }
  }
  return nearest_point;
}

int Course::findNearestIndex(double x, double y) const
{
  double min_distance = std::numeric_limits<double>::max();
  int nearest_index = 0;
  for (size_t i = 0; i < center_lane_.size(); ++i)
  {
    double distance = std::hypot(center_lane_[i].first - x, center_lane_[i].second - y);
    if (distance < min_distance)
    {
      min_distance = distance;
      nearest_index = i;
    }
  }
  return nearest_index;
}

std::pair<double, double> Course::getNextTargetPoint(double x, double y, double th, double lookahead_distance)
{
  int nearest_index = findNearestIndex(x, y);
  for (size_t i = nearest_index; i < center_lane_.size(); ++i)
  {
    double dx = center_lane_[i].first - x;
    double dy = center_lane_[i].second - y;
    double distance = std::hypot(dx, dy);
    if (distance >= lookahead_distance)
    {
      return center_lane_[i];
    }
  }
  return center_lane_.back();
}