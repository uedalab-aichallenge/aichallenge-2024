#include "dwa_node.hpp"

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <memory>

#include "robot.hpp"
#include "utils.hpp"

DWANode::DWANode()
    : Node("dwa_node") {
  // パブリッシャの初期化
  robot_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("robot_marker", 10);
  obstacles_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacles_marker", 10);
  path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("planned_path", 10);
  opt_path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("optimal_path", 10);
  multiple_paths_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("multiple_paths", 10);
  goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("goal_marker", 10);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("dwa_velocity", 10);
  pub_cmd_ = this->create_publisher<AckermannControlCommand>("output/control_cmd", 10);
  pub_raw_cmd_ = this->create_publisher<AckermannControlCommand>("output/raw_control_cmd", 10);

  // サブスクライバの初期化
  pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "robot_pose", 10, std::bind(&DWANode::poseCallback, this, std::placeholders::_1));
  sub_kinematics_ = create_subscription<Odometry>(
      "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "robot_velocity", 10, std::bind(&DWANode::velocityCallback, this, std::placeholders::_1));

  // タイマーの初期化
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(params_.DT * 1000)),
      std::bind(&DWANode::timerCallback, this));

  this->declare_parameter("ROBOT_INIT_X", 5.0);
  this->declare_parameter("ROBOT_INIT_Y", -5.0);
  this->declare_parameter("ROBOT_INIT_TH", 2.7);
  this->declare_parameter("DT", 0.1);
  this->declare_parameter("MAX_SPEED", 5.55556);
  this->declare_parameter("MIN_SPEED", 0.0);
  this->declare_parameter("MAX_YAWRATE", 3.14);
  this->declare_parameter("MIN_YAWRATE", -3.14);
  this->declare_parameter("MAX_ACCEL", 1.0);
  this->declare_parameter("MAX_DYAWRATE", 3.0);
  this->declare_parameter("V_RESOLUTION", 0.04);
  this->declare_parameter("YAWRATE_RESOLUTION", 0.04);
  this->declare_parameter("PREDICT_TIME", 2.0);
  this->declare_parameter("ROBOT_RADIUS", 0.2);
  this->declare_parameter("WEIGHT_ANGLE", 0.04);
  this->declare_parameter("WEIGHT_VELOCITY", 1.2);
  this->declare_parameter("WEIGHT_OBSTACLE", 1000.0);
  this->declare_parameter("WEIGHT_DISTANCE", 1.0);
  this->declare_parameter("LOOKAHEAD_DISTANCE", 5.0);
  this->declare_parameter("X_OFFSET", 89633.15625);
  this->declare_parameter("Y_OFFSET", 43127.796875);
  this->declare_parameter("OBS_SIZE", 0.3);
  this->declare_parameter("STEERING_TIRE_ANGLE_GAIN", 1.0);
  this->declare_parameter("SPEED_PROPORTIONAL_GAIN", 1.0);
  this->declare_parameter<std::string>("LEFT_LANE_BOUND_FILE", "/aichallenge/workspace/src/aichallenge_submit/dwa/csv_files/outer_track_interpolated.csv");
  this->declare_parameter<std::string>("RIGHT_LANE_BOUND_FILE", "/aichallenge/workspace/src/aichallenge_submit/dwa/csv_files/inner_track_interpolated.csv");
  this->declare_parameter<std::string>("CENTER_LANE_LINE_FILE", "/aichallenge/workspace/src/aichallenge_submit/dwa/csv_files/center_lane_line.csv");

  params_.ROBOT_INIT_X = this->get_parameter("ROBOT_INIT_X").as_double();
  params_.ROBOT_INIT_Y = this->get_parameter("ROBOT_INIT_Y").as_double();
  params_.ROBOT_INIT_TH = this->get_parameter("ROBOT_INIT_TH").as_double();
  params_.DT = this->get_parameter("DT").as_double();
  params_.MAX_SPEED = this->get_parameter("MAX_SPEED").as_double();
  params_.MIN_SPEED = this->get_parameter("MIN_SPEED").as_double();
  params_.MAX_YAWRATE = this->get_parameter("MAX_YAWRATE").as_double();
  params_.MIN_YAWRATE = this->get_parameter("MIN_YAWRATE").as_double();
  params_.MAX_ACCEL = this->get_parameter("MAX_ACCEL").as_double();
  params_.MAX_DYAWRATE = this->get_parameter("MAX_DYAWRATE").as_double();
  params_.V_RESOLUTION = this->get_parameter("V_RESOLUTION").as_double();
  params_.YAWRATE_RESOLUTION = this->get_parameter("YAWRATE_RESOLUTION").as_double();
  params_.PREDICT_TIME = this->get_parameter("PREDICT_TIME").as_double();
  params_.ROBOT_RADIUS = this->get_parameter("ROBOT_RADIUS").as_double();
  params_.WEIGHT_ANGLE = this->get_parameter("WEIGHT_ANGLE").as_double();
  params_.WEIGHT_VELOCITY = this->get_parameter("WEIGHT_VELOCITY").as_double();
  params_.WEIGHT_OBSTACLE = this->get_parameter("WEIGHT_OBSTACLE").as_double();
  params_.WEIGHT_DISTANCE = this->get_parameter("WEIGHT_DISTANCE").as_double();
  params_.LOOKAHEAD_DISTANCE = this->get_parameter("LOOKAHEAD_DISTANCE").as_double();
  params_.X_OFFSET = this->get_parameter("X_OFFSET").as_double();
  params_.Y_OFFSET = this->get_parameter("Y_OFFSET").as_double();
  params_.OBS_SIZE = this->get_parameter("OBS_SIZE").as_double();
  params_.SPEED_PROPORTIONAL_GAIN = this->get_parameter("SPEED_PROPORTIONAL_GAIN").as_double();
  params_.STEERING_TIRE_ANGLE_GAIN = this->get_parameter("STEERING_TIRE_ANGLE_GAIN").as_double();
  params_.LEFT_LANE_BOUND_FILE = this->get_parameter("LEFT_LANE_BOUND_FILE").as_string();
  params_.RIGHT_LANE_BOUND_FILE = this->get_parameter("RIGHT_LANE_BOUND_FILE").as_string();
  params_.CENTER_LANE_LINE_FILE = this->get_parameter("CENTER_LANE_LINE_FILE").as_string();
  // std::cout << "!!!!!!!!!!!!!weight_angle: " << params_.WEIGHT_ANGLE << std::endl;

  // 障害物のロード
  std::vector<std::string> csv_files = {params_.LEFT_LANE_BOUND_FILE, params_.RIGHT_LANE_BOUND_FILE};
  obstacles_ = loadObstacles(csv_files, params_);
  controller_ = std::make_unique<Controller>(params_);

  initMarkers();
}

void DWANode::initMarkers() {
  // ロボットマーカーの初期化
  robot_marker_.header.frame_id = "map";
  robot_marker_.ns = "robot";
  robot_marker_.id = 0;
  robot_marker_.type = visualization_msgs::msg::Marker::CYLINDER;
  robot_marker_.action = visualization_msgs::msg::Marker::ADD;
  robot_marker_.scale.x = 0.5 * 2;
  robot_marker_.scale.y = 0.5 * 2;
  robot_marker_.scale.z = 0.5;  // シリンダーの高さ
  robot_marker_.color.a = 1.0;
  robot_marker_.color.r = 0.0;
  robot_marker_.color.g = 1.0;
  robot_marker_.color.b = 0.0;

  // 障害物マーカーアレイの初期化
  obstacles_marker_.markers.clear();
  for (size_t i = 0; i < obstacles_.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "obstacles";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = obstacles_[i].getSize() * 2;
    marker.scale.y = obstacles_[i].getSize() * 2;
    marker.scale.z = 0.5;
    marker.pose.position.x = obstacles_[i].getX();
    marker.pose.position.y = obstacles_[i].getY();
    marker.pose.position.z = 0.25;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    obstacles_marker_.markers.push_back(marker);
  }

  // 計画されたパスマーカーの初期化
  path_marker_.header.frame_id = "map";
  path_marker_.ns = "planned_path";
  path_marker_.id = 0;
  path_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker_.action = visualization_msgs::msg::Marker::ADD;
  path_marker_.scale.x = 0.05;
  path_marker_.color.a = 1.0;
  path_marker_.color.r = 0.0;
  path_marker_.color.g = 0.0;
  path_marker_.color.b = 1.0;

  // 最適パスマーカーの初期化
  opt_path_marker_.header.frame_id = "map";
  opt_path_marker_.ns = "optimal_path";
  opt_path_marker_.id = 1;
  opt_path_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  opt_path_marker_.action = visualization_msgs::msg::Marker::ADD;
  opt_path_marker_.scale.x = 0.1;
  opt_path_marker_.color.a = 1.0;
  opt_path_marker_.color.r = 1.0;
  opt_path_marker_.color.g = 1.0;
  opt_path_marker_.color.b = 0.0;

  // ローカルパスマーカーの初期化
  local_path_marker_.header.frame_id = "map";
  local_path_marker_.ns = "local_path";
  local_path_marker_.id = 2;
  local_path_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  local_path_marker_.action = visualization_msgs::msg::Marker::ADD;
  local_path_marker_.scale.x = 0.05;
  local_path_marker_.color.a = 1.0;
  local_path_marker_.color.r = 0.0;
  local_path_marker_.color.g = 1.0;
  local_path_marker_.color.b = 1.0;

  // ゴールマーカーの初期化
  goal_marker_.header.frame_id = "map";
  goal_marker_.ns = "goal";
  goal_marker_.id = 0;
  goal_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  goal_marker_.action = visualization_msgs::msg::Marker::ADD;
  goal_marker_.scale.x = 0.5;
  goal_marker_.scale.y = 0.5;
  goal_marker_.scale.z = 0.3;
  goal_marker_.color.a = 1.0;
  goal_marker_.color.r = 0.0;
  goal_marker_.color.g = 1.0;
  goal_marker_.color.b = 0.0;
  goal_marker_.pose.orientation.w = 1.0;  // 中立の向き
}

void DWANode::poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
  current_pose_ = msg;
}

void DWANode::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  current_velocity_ = msg;
}

void DWANode::timerCallback() {
  auto start_time = std::chrono::high_resolution_clock::now();
  if (odometry_ == nullptr) {
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

  if (!traj_opt.empty()) {
    // Publish cmd_vel as before
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = controller_->getRobot().getUV();
    cmd_vel_msg.angular.z = controller_->getRobot().getUTh();
    cmd_vel_pub_->publish(cmd_vel_msg);

    // Compute steering tire angle based on the optimal path
    Path opt_path = traj_opt.back();
    size_t path_size = opt_path.getX().size();
    if (path_size >= 2) {
      double dx = opt_path.getX()[path_size - 1] - opt_path.getX()[path_size - 2];
      double dy = opt_path.getY()[path_size - 1] - opt_path.getY()[path_size - 2];
      double desired_yaw = std::atan2(dy, dx);
      double current_yaw =   controller_->getRobot().getTh();
      double yaw_error = desired_yaw - current_yaw;

      yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

      double steering_angle = -params_.STEERING_TIRE_ANGLE_GAIN * yaw_error;
      ackermann_cmd.longitudinal.speed = cmd_vel_msg.linear.x;
      ackermann_cmd.longitudinal.acceleration = 1.0;  // Adjust as needed
      ackermann_cmd.lateral.steering_tire_angle = steering_angle;

      // 目標速度と現在の速度の差を計算
      double speed_error = controller_->getRobot().getUV() - odometry_->twist.twist.linear.x;
      // 比例制御による加速度の計算
      double acceleration = params_.SPEED_PROPORTIONAL_GAIN * speed_error;

      std::cout << "speed_error: " << speed_error << std::endl;
      std::cout << "acceleration: " << acceleration << std::endl;
      ackermann_cmd.longitudinal.acceleration = 1.0;
      // ackermann_cmd.longitudinal.acceleration = acceleration;

      pub_cmd_->publish(ackermann_cmd);
      AckermannControlCommand raw_cmd = ackermann_cmd;
      raw_cmd.lateral.steering_tire_angle /= params_.STEERING_TIRE_ANGLE_GAIN;  // Invert the gain for raw angle
      pub_raw_cmd_->publish(raw_cmd);
    } else {
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
void DWANode::updateMarkers(const std::vector<Path> &traj_opt, const std::vector<std::vector<Path>> &traj_paths, const std::vector<double> &traj_x, const std::vector<double> &traj_y, const std::vector<double> &traj_th, const std::vector<double> &traj_g_x, const std::vector<double> &traj_g_y) {
  // ロボットのマーカー更新
  Robot robot = controller_->getRobot();
  robot_marker_.pose.position.x = robot.getX();
  robot_marker_.pose.position.y = robot.getY();
  robot_marker_.pose.orientation.z = std::sin(robot.getTh() / 2.0);
  robot_marker_.pose.orientation.w = std::cos(robot.getTh() / 2.0);

  // 計画されたパスのマーカー更新
  path_marker_.points.clear();
  for (size_t i = 0; i < traj_x.size(); ++i) {
    geometry_msgs::msg::Point p;
    p.x = traj_x[i];
    p.y = traj_y[i];
    p.z = 0.0;
    path_marker_.points.push_back(p);
  }

  // 最適パスのマーカー更新
  if (!traj_opt.empty()) {
    const Path &opt_path = traj_opt.back();
    opt_path_marker_.points.clear();
    for (size_t i = 0; i < opt_path.getX().size(); ++i) {
      geometry_msgs::msg::Point p;
      p.x = opt_path.getX()[i];
      p.y = opt_path.getY()[i];
      p.z = 0.0;
      opt_path_marker_.points.push_back(p);
    }

    // ゴールマーカーの更新
    goal_marker_.pose.position.x = traj_g_x.back();
    goal_marker_.pose.position.y = traj_g_y.back();
  }

  // 複数のパスのマーカー更新
  if (!traj_paths.empty()) {
    visualization_msgs::msg::MarkerArray multiple_paths_marker_array;
    const std::vector<Path> &latest_paths = traj_paths.back();
    for (size_t idx = 0; idx < latest_paths.size(); ++idx) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.ns = "multiple_paths_" + std::to_string(idx);
      marker.id = static_cast<int>(idx);
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.02;  // パスの太さを調整
      marker.color.a = 1.0;
      marker.color.r = static_cast<float>((idx % 255)) / 255.0f;
      marker.color.g = static_cast<float>((idx * 50) % 255) / 255.0f;
      marker.color.b = static_cast<float>((idx * 80) % 255) / 255.0f;

      marker.points.clear();
      for (size_t i = 0; i < latest_paths[idx].getX().size(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = latest_paths[idx].getX()[i];
        p.y = latest_paths[idx].getY()[i];
        p.z = 0.0;
        marker.points.push_back(p);
      }

      multiple_paths_marker_array.markers.push_back(marker);
    }
    // 複数パスのマーカーを保存
    multiple_paths_marker_array_ = multiple_paths_marker_array;
  }
}

void DWANode::publishMarkers() {
  // 現在の時刻を取得
  rclcpp::Time now = this->get_clock()->now();

  // ロボットマーカーのパブリッシュ
  robot_marker_.header.stamp = now;
  robot_marker_pub_->publish(robot_marker_);

  // 障害物マーカーのパブリッシュ
  obstacles_marker_pub_->publish(obstacles_marker_);

  // 計画されたパスマーカーのパブリッシュ
  path_marker_.header.stamp = now;
  path_marker_pub_->publish(path_marker_);

  // 最適パスマーカーのパブリッシュ
  if (!opt_path_marker_.points.empty()) {
    opt_path_marker_.header.stamp = now;
    opt_path_marker_pub_->publish(opt_path_marker_);
  }

  // ゴールマーカーのパブリッシュ
  goal_marker_.header.stamp = now;
  goal_marker_pub_->publish(goal_marker_);

  // 複数パスのマーカーをパブリッシュ
  if (!multiple_paths_marker_array_.markers.empty()) {
    for (auto &marker : multiple_paths_marker_array_.markers) {
      marker.header.stamp = now;
    }
    multiple_paths_pub_->publish(multiple_paths_marker_array_);
  }
}