#ifndef CALIBRATION_NODE_HPP
#define CALIBRATION_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "head_control/dynamixel_driver.hpp"

class CalibrationNode : public rclcpp::Node
{
    public:
        CalibrationNode();
        ~CalibrationNode();
        void run();

    private:
        struct JointCalib
        {
            int32_t min;
            int32_t max;
            int32_t center;
        };

        JointCalib calibrate_joint(const std::string & name, int id);
        void save_calib(const JointCalib &pitch, const JointCalib &yaw);

        DynamixelDriver driver_;
};

#endif