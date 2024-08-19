#include "goal_pose_setter_node.hpp"

GoalPosePublisher::GoalPosePublisher() : Node("goal_pose_publisher"),
    pitstop_flag_(false)
{
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    ekf_trigger_client_ = this->create_client<std_srvs::srv::SetBool>("/localization/trigger_node");
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", qos);
    route_state_subscriber_ = this->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
        "/planning/mission_planning/route_state",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
        std::bind(&GoalPosePublisher::route_state_callback, this, std::placeholders::_1));
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/localization/kinematic_state", 1,
        std::bind(&GoalPosePublisher::odometry_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(300),
        std::bind(&GoalPosePublisher::on_timer, this));

    sub_pitstop_status_ = create_subscription<std_msgs::msg::Int32>(
        "aichallenge/pitstop/condition", 1, [this](const std_msgs::msg::Int32::SharedPtr msg) {
            if (msg->data > 500){
                pitstop_flag_ = true;
            }
            else {
                pitstop_flag_ = false;
            }
        }
    );

    this->declare_parameter("goal.position.x", 21920.2);
    this->declare_parameter("goal.position.y", 51741.1);
    this->declare_parameter("goal.position.z", 0.0);
    this->declare_parameter("goal.orientation.x", 0.0);
    this->declare_parameter("goal.orientation.y", 0.0);
    this->declare_parameter("goal.orientation.z", 0.645336);
    this->declare_parameter("goal.orientation.w", 0.763899);

    this->declare_parameter("half_goal.position.x", 89657.0);
    this->declare_parameter("half_goal.position.y", 43175.0);
    this->declare_parameter("half_goal.position.z", -28.0);
    this->declare_parameter("half_goal.orientation.x", 0.0);
    this->declare_parameter("half_goal.orientation.y", 0.0);
    this->declare_parameter("half_goal.orientation.z", -0.9);
    this->declare_parameter("half_goal.orientation.w", 0.25);

    this->declare_parameter("pitstop_goal.position.x", 89626.625);
    this->declare_parameter("pitstop_goal.position.y", 43134.484375);
    this->declare_parameter("pitstop_goal.position.z", -28.0);
    this->declare_parameter("pitstop_goal.orientation.x", 0.0);
    this->declare_parameter("pitstop_goal.orientation.y", 0.0);
    this->declare_parameter("pitstop_goal.orientation.z", 0.8583754343434447);
    this->declare_parameter("pitstop_goal.orientation.w", 0.5130220401851199);

    this->declare_parameter("goal_range", 10.0);

    goal_position_.position.x = this->get_parameter("goal.position.x").as_double();
    goal_position_.position.y = this->get_parameter("goal.position.y").as_double();
    goal_position_.position.z = this->get_parameter("goal.position.z").as_double();
    goal_position_.orientation.x = this->get_parameter("goal.orientation.x").as_double();
    goal_position_.orientation.y = this->get_parameter("goal.orientation.y").as_double();
    goal_position_.orientation.z = this->get_parameter("goal.orientation.z").as_double();
    goal_position_.orientation.w = this->get_parameter("goal.orientation.w").as_double();


    half_goal_position_.position.x = this->get_parameter("half_goal.position.x").as_double();
    half_goal_position_.position.y = this->get_parameter("half_goal.position.y").as_double();
    half_goal_position_.position.z = this->get_parameter("half_goal.position.z").as_double();
    half_goal_position_.orientation.x = this->get_parameter("half_goal.orientation.x").as_double();
    half_goal_position_.orientation.y = this->get_parameter("half_goal.orientation.y").as_double();
    half_goal_position_.orientation.z = this->get_parameter("half_goal.orientation.z").as_double();
    half_goal_position_.orientation.w = this->get_parameter("half_goal.orientation.w").as_double();

    pitstop_goal_position_.position.x = this->get_parameter("pitstop_goal.position.x").as_double();
    pitstop_goal_position_.position.y = this->get_parameter("pitstop_goal.position.y").as_double();
    pitstop_goal_position_.position.z = this->get_parameter("pitstop_goal.position.z").as_double();
    pitstop_goal_position_.orientation.x = this->get_parameter("pitstop_goal.orientation.x").as_double();
    pitstop_goal_position_.orientation.y = this->get_parameter("pitstop_goal.orientation.y").as_double();
    pitstop_goal_position_.orientation.z = this->get_parameter("pitstop_goal.orientation.z").as_double();
    pitstop_goal_position_.orientation.w = this->get_parameter("pitstop_goal.orientation.w").as_double();

    goal_range_ = this->get_parameter("goal_range").as_double();
}

void GoalPosePublisher::on_timer()
{
    if (++delay_count_ <= 10) {
        return;
    }

    if (!stop_initializing_pose_) {
        if (ekf_trigger_client_->service_is_ready()) {
            const auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
            req->data = true;
            ekf_trigger_client_->async_send_request(req, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
            {
                stop_initializing_pose_ = future.get()->success;
                delay_count_ = 0;
                RCLCPP_INFO(this->get_logger(), "Complete localization trigger");
            });
            RCLCPP_INFO(this->get_logger(), "Call localization trigger");
        }
        return;
    }

    if (!stop_streaming_goal_pose_)
    {
        auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "map";
        msg->pose.position.x = this->get_parameter("goal.position.x").as_double();
        msg->pose.position.y = this->get_parameter("goal.position.y").as_double();
        msg->pose.position.z = this->get_parameter("goal.position.z").as_double();
        msg->pose.orientation.x = this->get_parameter("goal.orientation.x").as_double();
        msg->pose.orientation.y = this->get_parameter("goal.orientation.y").as_double();
        msg->pose.orientation.z = this->get_parameter("goal.orientation.z").as_double();
        msg->pose.orientation.w = this->get_parameter("goal.orientation.w").as_double();

        goal_publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Publishing goal pose");
    }
}

void GoalPosePublisher::route_state_callback(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg)
{
    if (msg->state >= autoware_adapi_v1_msgs::msg::RouteState::SET)
    {
        stop_streaming_goal_pose_ = true;
        RCLCPP_INFO(this->get_logger(), "Stop streaming goal pose");
    }
    is_started_ = true;
}

int GoalPosePublisher::lap_counter(double distance){
    static int lap_cnt = 1;
    static bool reached_flag = false;
    static bool once_flag = false;

    double distance_th = 1.0;

    if (distance < distance_th){
        if(not reached_flag){
            lap_cnt++;
        }
        reached_flag = true;
    }
    else{
        reached_flag = false;
    }

    return lap_cnt;
}

void GoalPosePublisher::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    int lapcnt = lap_counter(tier4_autoware_utils::calcDistance2d(msg->pose.pose, goal_position_));

    // std::cerr << lapcnt << "\n";

    if (!is_started_)
        return;

    if (pitstop_flag_ == true && pitstop_goal_pose_published_ == true){
        return;
    }

    if (pitstop_flag_ == false && pitstop_goal_pose_published_ == true){
        pitstop_goal_pose_published_ = false;
        auto goal_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
        goal_pose->header.stamp = this->get_clock()->now();
        goal_pose->header.frame_id = "map";
        goal_pose->pose = half_goal_position_;

        goal_publisher_->publish(*goal_pose);
        RCLCPP_INFO(this->get_logger(), "Publishing half goal pose for loop");
        half_goal_pose_published_ = true;
    }

    // Publish half goal pose for pitstop
    if(pitstop_flag_ == true && half_goal_pose_published_ == false && pitstop_goal_pose_published_ == false &&
        tier4_autoware_utils::calcDistance2d(msg->pose.pose, goal_position_) < goal_range_ && 6 > lapcnt)
    {
        auto goal_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
        goal_pose->header.stamp = this->get_clock()->now();
        goal_pose->header.frame_id = "map";
        goal_pose->pose = pitstop_goal_position_;

        goal_publisher_->publish(*goal_pose);
        RCLCPP_INFO(this->get_logger(), "Publishing half goal pose for loop");
        pitstop_goal_pose_published_ = true;
        
        return;
    }

    // Publish half goal pose for loop
    if(half_goal_pose_published_ == false &&
        tier4_autoware_utils::calcDistance2d(msg->pose.pose, goal_position_) < goal_range_) 
    {
        auto goal_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
        goal_pose->header.stamp = this->get_clock()->now();
        goal_pose->header.frame_id = "map";
        goal_pose->pose = half_goal_position_;

        goal_publisher_->publish(*goal_pose);
        RCLCPP_INFO(this->get_logger(), "Publishing half goal pose for loop");
        half_goal_pose_published_ = true;
    }
    // Publish goal pose for loop
    if (half_goal_pose_published_ == true &&
        tier4_autoware_utils::calcDistance2d(msg->pose.pose, half_goal_position_) < goal_range_) 
    {
        auto goal_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
        goal_pose->header.stamp = this->get_clock()->now();
        goal_pose->header.frame_id = "map";
        goal_pose->pose = goal_position_;

        goal_publisher_->publish(*goal_pose);
        RCLCPP_INFO(this->get_logger(), "Publishing goal pose for loop");
        half_goal_pose_published_ = false;
    }
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPosePublisher>());
    rclcpp::shutdown();
    return 0;
}
