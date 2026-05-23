#include "head_control/calibration_node.hpp"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

CalibrationNode::CalibrationNode() : Node("calibration_node")
{
    try 
    {
        driver_.init();
    } 
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(get_logger(), "Driver init failed: %s", e.what());
        throw;
    }
}

CalibrationNode::~CalibrationNode()
{

}

CalibrationNode::JointCalib CalibrationNode::calibrate_joint(const std::string & name, int id)
{
    RCLCPP_INFO(get_logger(), "=== Calibrating %s ===", name.c_str());

    int32_t pitch_ticks, yaw_ticks;

    // Calibration process similar to lerobot by moving to extremes
    RCLCPP_INFO(get_logger(), "Move %s to one extreme, then press Enter", name.c_str());
    std::cin.get();
    driver_.read_current_positions(pitch_ticks, yaw_ticks);
    int32_t pos_a = (id == DynamixelDriver::PITCH_ID) ? pitch_ticks : yaw_ticks;

    RCLCPP_INFO(get_logger(), "Move %s to other extreme, then press Enter", name.c_str());
    std::cin.get();
    driver_.read_current_positions(pitch_ticks, yaw_ticks);
    int32_t pos_b = (id == DynamixelDriver::PITCH_ID) ? pitch_ticks : yaw_ticks;

    JointCalib calib;
    calib.min    = std::min(pos_a, pos_b);
    calib.max    = std::max(pos_a, pos_b);
    calib.center = (calib.min + calib.max) / 2;

    RCLCPP_INFO(get_logger(), "%s — min: %d  max: %d  center: %d",
        name.c_str(), calib.min, calib.max, calib.center);

    return calib;
}

void CalibrationNode::save_calib(const JointCalib & pitch, const JointCalib & yaw)
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "/**" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "ros__parameters" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "pitch" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "min"    << YAML::Value << pitch.min;
    out << YAML::Key << "max"    << YAML::Value << pitch.max;
    out << YAML::Key << "center" << YAML::Value << pitch.center;
    out << YAML::EndMap;
    out << YAML::Key << "yaw" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "min"    << YAML::Value << yaw.min;
    out << YAML::Key << "max"    << YAML::Value << yaw.max;
    out << YAML::Key << "center" << YAML::Value << yaw.center;
    out << YAML::EndMap;
    out << YAML::EndMap;
    out << YAML::EndMap;  
    out << YAML::EndMap; 

    std::string path = ament_index_cpp::get_package_share_directory("head_control") + "/config/calib.yaml";
    std::ofstream file(path);
    if (!file) 
    {
        RCLCPP_ERROR(get_logger(), "Failed to open %s for writing", path.c_str());
        return;
    }
    file << out.c_str();
    RCLCPP_INFO(get_logger(), "Saved to %s", path.c_str());
}

void CalibrationNode::run()
{
    // Calibrate both the joints of the head
    JointCalib pitch = calibrate_joint("PITCH", DynamixelDriver::PITCH_ID);
    JointCalib yaw = calibrate_joint("YAW", DynamixelDriver::YAW_ID);

    //Enable torque after calibration
    driver_.enable_torque(DynamixelDriver::PITCH_ID);
    driver_.enable_torque(DynamixelDriver::YAW_ID);

    // Move to the center
    RCLCPP_INFO(get_logger(), "Moving to center ...");
    driver_.write_goal_positions(pitch.center, yaw.center);

    // Save calibration data
    save_calib(pitch, yaw);
    RCLCPP_INFO(get_logger(), "Calibration saved!");

}



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalibrationNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
