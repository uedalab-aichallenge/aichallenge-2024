#include "dwa_node.hpp"
#include "utils.hpp"
#include "robot.hpp"
#include <memory>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

// ... existing code ...

void DWANode::timerCallback()
{
  auto start_time = std::chrono::high_resolution_clock::now();
  if (odometry_ == nullptr)
  {
    // RCLCPP_ERROR(this->get_logger(), "Robot state data not received");
    return;
  }

  // Update robot state from odometry
  controller_->getRobot().setX(odometry_->pose.pose.position.x);
  controller_->getRobot().setY(odometry_->pose.pose.position.y);
  
  // Extract yaw from quaternion
  double siny_cosp = 2.0 * (odometry_->pose.pose.orientation.w * odometry_->pose.pose.orientation.z + 
                            odometry_->pose.pose.orientation.x * odometry_->pose.pose.orientation.y);
  double cosy_cosp = 1.0 - 2.0 * (odometry_->pose.pose.orientation.y * odometry_->pose.pose.orientation.y + 
                                   odometry_->pose.pose.orientation.z * odometry_->pose.pose.orientation.z);
  double th = std::atan2(siny_cosp, cosy_cosp);
  controller_->getRobot().setTh(th);
  
  // Set velocity from odometry
  controller_->getRobot().setUV(odometry_->twist.twist.linear.x);
  controller_->getRobot().setUTh(odometry_->twist.twist.angular.z);

  // Execute controller step
  auto result = controller_->runStep(obstacles_);
  std::vector<double> traj_x = std::get<0>(result);
  std::vector<double> traj_y = std::get<1>(result);
  std::vector<double> traj_th = std::get<2>(result);
  std::vector<std::vector<Path>> traj_paths = std::get<3>(result);
  std::vector<Path> traj_opt = std::get<4>(result);
  std::vector<double> traj_g_x = std::get<5>(result);
  std::vector<double> traj_g_y = std::get<6>(result);
  Course course = std::get<7>(result);

  // Update visualization markers
  updateMarkers(traj_opt, traj_paths, traj_x, traj_y, traj_th, traj_g_x, traj_g_y);

  // Create AckermannControlCommand
  AckermannControlCommand ackermann_cmd;
  ackermann_cmd.stamp = this->get_clock()->now();

  if (!traj_opt.empty())
  {
    // Publish cmd_vel as before
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = controller_->getRobot().getUV();
    cmd_vel_msg.angular.z = controller_->getRobot().getUTh();
    cmd_vel_pub_->publish(cmd_vel_msg);

    // Compute steering tire angle based on the optimal path
    Path opt_path = traj_opt.back();
    size_t path_size = opt_path.getX().size();
    if (path_size >= 2)
    {
      double dx = opt_path.getX()[path_size - 1] - opt_path.getX()[path_size - 2];
      double dy = opt_path.getY()[path_size - 1] - opt_path.getY()[path_size - 2];
      double desired_yaw = std::atan2(dy, dx);
      double current_yaw = controller_->getRobot().getTh();
      double yaw_error = desired_yaw - current_yaw;

      // Normalize yaw_error to [-pi, pi]
      yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

      // Compute steering angle
      double steering_angle = params_.STEERING_TIRE_ANGLE_GAIN * yaw_error;

      // Populate AckermannControlCommand
      ackermann_cmd.longitudinal.speed = cmd_vel_msg.linear.x;
      ackermann_cmd.longitudinal.acceleration = 1.0; // Adjust as needed
      ackermann_cmd.lateral.steering_tire_angle = steering_angle;

      // Publish the command
      pub_cmd_->publish(ackermann_cmd);

      // Populate AckermannControlCommand for raw command
      AckermannControlCommand raw_cmd = ackermann_cmd;
      raw_cmd.lateral.steering_tire_angle /= params_.STEERING_TIRE_ANGLE_GAIN; // Invert the gain for raw angle
      pub_raw_cmd_->publish(raw_cmd);
    }
    else
    {
      // Handle cases with insufficient path points
      ackermann_cmd.longitudinal.speed = 0.0;
      ackermann_cmd.longitudinal.acceleration = -10.0;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "Insufficient path points");
      pub_cmd_->publish(ackermann_cmd);
    }
  }

  // Publish visualization markers
  publishMarkers();

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
  RCLCPP_INFO(this->get_logger(), "Time taken for one step: %ld ms", duration);
} 