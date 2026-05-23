#include "head_control/dynamixel_driver.hpp"

DynamixelDriver::DynamixelDriver()
{
    
}

DynamixelDriver::~DynamixelDriver()
{
    shutdown();
}

void DynamixelDriver::init()
{
    port_handler_ = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    //Open port for communication with u2d2
    auto dxl_comm_res = port_handler_ -> openPort();
    if(!dxl_comm_res)
    {
        throw std::runtime_error("Failed to open port");
    }

    // Set baudrate
    dxl_comm_res = port_handler_ -> setBaudRate(BAUDRATE);
    if(!dxl_comm_res)
    {
        throw std::runtime_error("Failed to set baudrate");
    }

    uint8_t dxl_error = 0;

    for(int id : {PITCH_ID, YAW_ID})
    {
        // Set each motor into Extended Position Control
        auto dxl_res = packet_handler_ ->write1ByteTxRx(
            port_handler_,
            id,
            ADDR_OPERATING_MODE,
            POSITION_CONTROL_MODE,
            &dxl_error
        );

        if(dxl_res != COMM_SUCCESS)
        {
            throw std::runtime_error("Failed to set Position control");
        }

        // Set each motor velocity profile
        dxl_res = packet_handler_->write4ByteTxRx(
            port_handler_,
            id,
            ADDR_PROFILE_VELOCITY,
            PROFILE_VELOCITY,
            &dxl_error
        );

        if(dxl_res != COMM_SUCCESS)
        {
            throw std::runtime_error("Failed to set profile velocity");
        }
    }
    
    sync_read_ = new dynamixel::GroupSyncRead(port_handler_, packet_handler_, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    sync_read_ ->addParam(PITCH_ID);
    sync_read_ ->addParam(YAW_ID);
    sync_write_ = new dynamixel::GroupSyncWrite(port_handler_, packet_handler_, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);


}

void DynamixelDriver::shutdown()
{
    uint8_t dxl_error = 0;

    // Disable torque for each motor
    for (int id : {PITCH_ID, YAW_ID}) {
        if (torque_enabled_[id - 1]) {
            packet_handler_->write1ByteTxRx(
                port_handler_, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, &dxl_error);
            torque_enabled_[id - 1] = false;
        }
    }
    delete sync_read_;
    delete sync_write_;
    port_handler_->closePort();
}

void DynamixelDriver::enable_torque(int id)
{
    uint8_t dxl_error = 0;
    auto dxl_res = packet_handler_->write1ByteTxRx(
        port_handler_, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE_VAL, &dxl_error);
    if (dxl_res != COMM_SUCCESS)
        throw std::runtime_error("Failed to enable torque for id " + std::to_string(id));
    torque_enabled_[id - 1] = true;
}

void DynamixelDriver::disable_torque(int id)
{
    uint8_t dxl_error = 0;
    auto dxl_res = packet_handler_->write1ByteTxRx(
        port_handler_, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, &dxl_error);
    if (dxl_res != COMM_SUCCESS)
        throw std::runtime_error("Failed to disable torque for id " + std::to_string(id));
    torque_enabled_[id - 1] = false;
}

bool DynamixelDriver::write_goal_positions(int32_t pitch_ticks, int32_t yaw_ticks)
{
    sync_write_->clearParam();

    uint8_t pitch_data[4] = {
        DXL_LOBYTE(DXL_LOWORD(pitch_ticks)),
        DXL_HIBYTE(DXL_LOWORD(pitch_ticks)),
        DXL_LOBYTE(DXL_HIWORD(pitch_ticks)),
        DXL_HIBYTE(DXL_HIWORD(pitch_ticks))
    };
    uint8_t yaw_data[4] = {
        DXL_LOBYTE(DXL_LOWORD(yaw_ticks)),
        DXL_HIBYTE(DXL_LOWORD(yaw_ticks)),
        DXL_LOBYTE(DXL_HIWORD(yaw_ticks)),
        DXL_HIBYTE(DXL_HIWORD(yaw_ticks))
    };

    sync_write_->addParam(PITCH_ID, pitch_data);
    sync_write_->addParam(YAW_ID, yaw_data);

    int result = sync_write_->txPacket();
    if (result != COMM_SUCCESS) 
    {
        return false;
    }
    return true;
}

bool DynamixelDriver::read_current_positions(int32_t &pitch_ticks, int32_t &yaw_ticks)
{
    int result = sync_read_->txRxPacket();
    if (result != COMM_SUCCESS) 
    {
        return false;
    }

    if (!sync_read_->isAvailable(PITCH_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) ||
        !sync_read_->isAvailable(YAW_ID,   ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) 
    {
        return false;
    }

    pitch_ticks = static_cast<int32_t>(sync_read_->getData(PITCH_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION));
    yaw_ticks   = static_cast<int32_t>(sync_read_->getData(YAW_ID,   ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION));

    return true;
}

