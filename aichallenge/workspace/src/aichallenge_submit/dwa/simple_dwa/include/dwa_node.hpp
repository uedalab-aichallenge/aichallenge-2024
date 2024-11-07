#pragma once

#include "controller.hpp"
#include "course.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "types.hpp" // Parameters構造体をインクルード
#include <memory> // 追加
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

class DWANode : public rclcpp::Node
{
public:
  DWANode();
private:
  void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timerCallback();
  void initMarkers();
  void publishMarkers();
  void updateMarkers(const std::vector<Path> &traj_opt, const std::vector<std::vector<Path>> &traj_paths, const std::vector<double> &traj_x, const std::vector<double> &traj_y, const std::vector<double> &traj_th, const std::vector<double> &traj_g_x, const std::vector<double> &traj_g_y);
  // コントローラー
  std::unique_ptr<Controller> controller_; // ポインタ型に変更
  Parameters params_;                      // 追加

  // 障害物
  std::vector<Obstacle> obstacles_;

  // サブスクライバ
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;

  // パブリッシャ
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr opt_path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr multiple_paths_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_raw_cmd_;

  // タイマー
  rclcpp::TimerBase::SharedPtr timer_;

  // ロボット状態
  geometry_msgs::msg::Pose::SharedPtr current_pose_;
  geometry_msgs::msg::Twist::SharedPtr current_velocity_;
  Odometry::SharedPtr odometry_;

  // マーカー
  visualization_msgs::msg::Marker robot_marker_;
  visualization_msgs::msg::MarkerArray obstacles_marker_;
  visualization_msgs::msg::Marker path_marker_;
  visualization_msgs::msg::Marker opt_path_marker_;
  visualization_msgs::msg::Marker local_path_marker_;
  visualization_msgs::msg::Marker goal_marker_;
  visualization_msgs::msg::MarkerArray multiple_paths_marker_array_;

  // その他

};
