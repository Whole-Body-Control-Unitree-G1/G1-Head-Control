#ifndef DYNAMIXEL_DRIVER_HPP
#define DYNAMIXEL_DRIVER_HPP
#include <cmath>
#include "dynamixel_sdk/dynamixel_sdk.h"

class DynamixelDriver
{
public:
    DynamixelDriver();
    ~DynamixelDriver();
    void init();
    void shutdown();

    // Torque control
    void enable_torque(int id);
    void disable_torque(int id);

    // Motion
    bool write_goal_positions(int32_t pitch_ticks, int32_t yaw_ticks);
    bool read_current_positions(int32_t &pitch_ticks, int32_t &yaw_ticks);

    // Exposed for tick math in nodes
    static constexpr double TICKS_PER_RAD = 4096.0 / (2.0 * M_PI);
    static constexpr int    PITCH_ID      = 1;
    static constexpr int    YAW_ID        = 2;

private:
    // Control table addresses (X-series, Protocol 2.0)
    static constexpr uint16_t ADDR_OPERATING_MODE   = 11;
    static constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
    static constexpr uint16_t ADDR_PROFILE_VELOCITY = 112;
    static constexpr uint16_t ADDR_GOAL_POSITION    = 116;
    static constexpr uint16_t ADDR_PRESENT_POSITION = 132;
    static constexpr uint16_t LEN_GOAL_POSITION     = 4;
    static constexpr uint16_t LEN_PRESENT_POSITION  = 4;

    // Operating mode values
    static constexpr uint8_t  POSITION_CONTROL_MODE = 4;
    static constexpr uint8_t  TORQUE_ENABLE_VAL     = 1;
    static constexpr uint8_t  TORQUE_DISABLE_VAL    = 0;

    // Hardware config
    static constexpr double   PROTOCOL_VERSION = 2.0;
    static constexpr int      BAUDRATE         = 57600;
    static constexpr char     DEVICE_NAME[]    = "/dev/ttyUSB0";
    static constexpr uint32_t PROFILE_VELOCITY = 100;

    dynamixel::PortHandler    *port_handler_  {nullptr};
    dynamixel::PacketHandler  *packet_handler_ {nullptr};
    dynamixel::GroupSyncWrite *sync_write_    {nullptr};
    dynamixel::GroupSyncRead  *sync_read_     {nullptr};

    // indexed by (id - 1): [0] = pitch, [1] = yaw
    bool torque_enabled_[2] {false, false};
};

#endif
