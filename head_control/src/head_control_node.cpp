#include "head_control/head_control_node.hpp"
#include <cmath>
#include <stdexcept>

HeadControlNode::HeadControlNode() : Node("head_control_node")
{
    declare_parameter("pitch.center", 0);
    declare_parameter("pitch.min",    0);
    declare_parameter("pitch.max",    0);
    declare_parameter("yaw.center",   0);
    declare_parameter("yaw.min",      0);
    declare_parameter("yaw.max",      0);

    pitch_center_ = get_parameter("pitch.center").as_int();
    pitch_min_    = get_parameter("pitch.min").as_int();
    pitch_max_    = get_parameter("pitch.max").as_int();
    yaw_center_   = get_parameter("yaw.center").as_int();
    yaw_min_      = get_parameter("yaw.min").as_int();
    yaw_max_      = get_parameter("yaw.max").as_int();

    RCLCPP_INFO(get_logger(), "Calibration loaded — pitch: [%d, %d, %d]  yaw: [%d, %d, %d]",
        pitch_min_, pitch_center_, pitch_max_, yaw_min_, yaw_center_, yaw_max_);

    try 
    {
        driver_.init();
        driver_.enable_torque(DynamixelDriver::PITCH_ID);
        driver_.enable_torque(DynamixelDriver::YAW_ID);
    } 
    catch (const std::exception & e) 
    {
        RCLCPP_ERROR(get_logger(), "Driver setup failed: %s", e.what());
        throw;
    }

    target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "head/target", 10,
        std::bind(&HeadControlNode::target_callback, this, std::placeholders::_1)
    );

    state_pub_ = create_publisher<sensor_msgs::msg::JointState>("head/state", 10);

    state_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&HeadControlNode::publish_state, this)
    );

    goal_pitch_pos_ = pitch_center_;
    goal_yaw_pos_   = yaw_center_;
    driver_.write_goal_positions(goal_pitch_pos_, goal_yaw_pos_);
    RCLCPP_INFO(get_logger(), "Head centered at startup");
}

HeadControlNode::~HeadControlNode() {}

int32_t HeadControlNode::rad_to_pos(double rad, int32_t center)
{
    return static_cast<int32_t>(std::round(center + rad * DynamixelDriver::TICKS_PER_RAD));
}

double HeadControlNode::pos_to_rad(int32_t pos, int32_t center)
{
    return static_cast<double>(pos - center) / DynamixelDriver::TICKS_PER_RAD;
}

int32_t HeadControlNode::clamp_pos(int32_t pos, int32_t lo, int32_t hi)
{
    return std::max(lo, std::min(hi, pos));
}

void HeadControlNode::target_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    bool got_pitch = false, got_yaw = false;
    double pitch_rad = 0.0, yaw_rad = 0.0;

    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "pitch" && i < msg->position.size()) {
            pitch_rad = msg->position[i];
            got_pitch = true;
        } else if (msg->name[i] == "yaw" && i < msg->position.size()) {
            yaw_rad = msg->position[i];
            got_yaw = true;
        }
    }

    if (!got_pitch && !got_yaw) {
        RCLCPP_WARN(get_logger(), "Received JointState with no 'pitch' or 'yaw' fields");
        return;
    }

    if (got_pitch)
        goal_pitch_pos_ = clamp_pos(rad_to_pos(-pitch_rad, pitch_center_), pitch_min_, pitch_max_);
    if (got_yaw)
        goal_yaw_pos_ = clamp_pos(rad_to_pos(yaw_rad, yaw_center_), yaw_min_, yaw_max_);

    driver_.write_goal_positions(goal_pitch_pos_, goal_yaw_pos_);
}

void HeadControlNode::publish_state()
{
    int32_t pitch_pos, yaw_pos;
    if (!driver_.read_current_positions(pitch_pos, yaw_pos))
        return;

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();
    msg.name         = {"pitch", "yaw"};
    msg.position     = {-pos_to_rad(pitch_pos, pitch_center_), pos_to_rad(yaw_pos, yaw_center_)};
    state_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadControlNode>());
    rclcpp::shutdown();
    return 0;
}
