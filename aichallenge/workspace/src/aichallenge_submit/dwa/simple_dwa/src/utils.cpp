#include "utils.hpp"
#include "obstacle.hpp"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
std::vector<double> minMaxNormalize(const std::vector<double> &data)
{
  std::vector<double> normalized_data;
  if (data.empty())
  {
    return normalized_data;
  }

  double max_data = *std::max_element(data.begin(), data.end());
  double min_data = *std::min_element(data.begin(), data.end());

  if (max_data - min_data == 0)
  {
    normalized_data.assign(data.size(), 0.0);
  }
  else
  {
    normalized_data.reserve(data.size());
    for (const auto &d : data)
    {
      normalized_data.push_back((d - min_data) / (max_data - min_data));
    }
  }

  return normalized_data;
}

double angleRangeCorrector(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2 * M_PI;
  }
  return angle;
}

std::vector<Obstacle> loadObstacles(const std::vector<std::string> &csv_files, const Parameters &params)
{
  std::vector<Obstacle> obstacles;
  for (const auto &file_path : csv_files)
  {
    std::ifstream file(file_path);
    if (!file.is_open())
    {
      // ファイルが見つからない場合はスキップ
      continue;
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
        double x = std::stod(x_str) - params.X_OFFSET;
        double y = std::stod(y_str) - params.Y_OFFSET;
        obstacles.emplace_back(x, y, params.OBS_SIZE); // サイズは必要に応じて調整
      }
      catch (...)
      {
        // 無効なデータ行はスキップ
        continue;
      }
    }
  }
  return obstacles;
}