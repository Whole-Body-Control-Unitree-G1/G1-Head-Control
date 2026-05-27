#ifndef HEAD_CONTROL_NODE_HPP
#define HEAD_CONTROL_NODE_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "head_control/dynamixel_driver.hpp"

class HeadControlNode : public rclcpp::Node
{
public:
    HeadControlNode();
    ~HeadControlNode();

private:
    static int32_t rad_to_pos(double rad, int32_t center);
    static double  pos_to_rad(int32_t pos, int32_t center);
    static int32_t clamp_pos(int32_t pos, int32_t lo, int32_t hi);
    int32_t        unwrap_pitch(int32_t pos) const;

    void target_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publish_state();

    DynamixelDriver driver_;

    // Calibration values loaded from calib.yaml
    int32_t pitch_center_ {0};
    int32_t pitch_min_    {0};
    int32_t pitch_max_    {0};
    int32_t yaw_center_   {0};
    int32_t yaw_min_      {0};
    int32_t yaw_max_      {0};

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    state_pub_;
    rclcpp::TimerBase::SharedPtr state_timer_;

    int32_t goal_pitch_pos_ {0};
    int32_t goal_yaw_pos_   {0};
};

#endif
