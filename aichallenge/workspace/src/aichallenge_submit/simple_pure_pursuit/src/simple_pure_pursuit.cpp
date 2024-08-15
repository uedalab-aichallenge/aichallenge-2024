#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>

namespace simple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SimplePurePursuit::SimplePurePursuit()
: Node("simple_pure_pursuit"),
  // initialize parameters
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  pitstop_flag_(false)
{
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);

  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });
  
  sub_pitstop_status_ = create_subscription<std_msgs::msg::Int32>(
    "aichallenge/pitstop/condition", 1, [this](const std_msgs::msg::Int32::SharedPtr msg) {
      pitstop_status_ = msg; 

      if (pitstop_status_->data > 500){
        pitstop_flag_ = true;
      }
      else {
        pitstop_flag_ = false;
      }
    }
  );

  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 30ms, std::bind(&SimplePurePursuit::onTimer, this));
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

void SimplePurePursuit::onTimer()
{
  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }

  size_t closest_traj_point_idx =
    findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

  // get closest trajectory point from current position
  TrajectoryPoint closest_traj_point = trajectory_->points.at(closest_traj_point_idx);

  // calc longitudinal speed and acceleration
  double target_longitudinal_vel =
    use_external_target_vel_ ? external_target_vel_ : closest_traj_point.longitudinal_velocity_mps;
  double current_longitudinal_vel = odometry_->twist.twist.linear.x;

  double pitstop_x = 89626.625;
  double pitstop_y = 43134.484375;

  // 停止位置への距離を計算
  double distance_to_final_point = std::hypot(
    pitstop_x - odometry_->pose.pose.position.x,
    pitstop_y - odometry_->pose.pose.position.y
  );
  
  double stop_threshold = 2.0; // 停止とみなす閾値
  double deceleration_distance = 20.0; // 減速を開始する距離

  if (distance_to_final_point < stop_threshold && pitstop_flag_) {
    // 停止位置に近いため停止する
    cmd.longitudinal.speed = 0.0;
    cmd.longitudinal.acceleration = -1000; // 減速
  } else if (distance_to_final_point < deceleration_distance && pitstop_flag_) {
    // 停止位置に近づいているため減速する
    double deceleration = (current_longitudinal_vel * current_longitudinal_vel) / (2.0 * distance_to_final_point);
    cmd.longitudinal.speed = std::max(0.0, current_longitudinal_vel - deceleration);
    cmd.longitudinal.acceleration = -deceleration;
  } else {
    // 通常の速度制御
    cmd.longitudinal.speed = target_longitudinal_vel;
    cmd.longitudinal.acceleration =
      speed_proportional_gain_ * (target_longitudinal_vel - current_longitudinal_vel);
  }

  // 横方向の制御を計算
  double lookahead_distance = lookahead_gain_ * target_longitudinal_vel + lookahead_min_distance_;
  double rear_x = odometry_->pose.pose.position.x -
                  wheel_base_ / 2.0 * std::cos(odometry_->pose.pose.orientation.z);
  double rear_y = odometry_->pose.pose.position.y -
                  wheel_base_ / 2.0 * std::sin(odometry_->pose.pose.orientation.z);
  
  auto lookahead_point_itr = std::find_if(
    trajectory_->points.begin() + closest_traj_point_idx, trajectory_->points.end(),
    [&](const TrajectoryPoint & point) {
      return std::hypot(point.pose.position.x - rear_x, point.pose.position.y - rear_y) >=
             lookahead_distance;
    });
  if (lookahead_point_itr == trajectory_->points.end()) {
    lookahead_point_itr = trajectory_->points.end() - 1;
  }
  
  double lookahead_point_x = lookahead_point_itr->pose.position.x;
  double lookahead_point_y = lookahead_point_itr->pose.position.y;

  double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
                 tf2::getYaw(odometry_->pose.pose.orientation);
  cmd.lateral.steering_tire_angle =
    std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);

  pub_cmd_->publish(cmd);
}


bool SimplePurePursuit::subscribeMessageAvailable()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory is not available");
    return false;
  }
  return true;
}
}  // namespace simple_pure_pursuit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
