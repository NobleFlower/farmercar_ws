#include "vehicle_controller/virtual_serial_handler.h"

#include <ros/ros.h>

namespace vehicle_controller
{
bool processVirtualSerialPacket(const std::vector<uint8_t>& packet, VehicleState& state, const std::string& portName)
{
    if (packet.size() != 8)
    {
        return false;
    }

    float speedValue = 0.0f;
    float steeringAngle = 90.0f;
    if (!parseVirtualSpeedPacket(packet, speedValue, steeringAngle))
    {
        return false;
    }

    const uint16_t rawSpeed = static_cast<uint16_t>(packet[3]) |
                              (static_cast<uint16_t>(packet[4]) << 8U);
    const uint16_t rawSteering = static_cast<uint16_t>(packet[5]) |
                                (static_cast<uint16_t>(packet[6]) << 8U);

    state.currentLowSpeedValue = speedValue;
    state.currentSteeringAngle = steeringAngle;
    ROS_INFO_STREAM("virtual_serial " << portName << " received: "
                    << "speed=" << std::fixed << std::setprecision(2)
                    << state.currentLowSpeedValue << " (raw=" << rawSpeed << ")"
                    << ", current angle=" << std::setprecision(1)
                    << state.currentSteeringAngle << " deg (raw=" << rawSteering << ")");

    return true;
}

bool sendLowSpeedMotionControl(int virtualFd, const VehicleState& state, CommandType commandType)
{
    if (!(state.lowSpeedMode &&
          (commandType == CommandType::MoveForward ||
           commandType == CommandType::MoveStop ||
           commandType == CommandType::MoveReverse)))
    {
        return false;
    }

    float targetSpeed = state.currentLowSpeedValue;
    float targetSteeringAngle = state.currentSteeringAngle;
    std::string motionState = "前进";

    if (commandType == CommandType::MoveStop)
    {
        targetSpeed = 0.0f;
        targetSteeringAngle = 90.0f;
        motionState = "停车";
    }
    else if (commandType == CommandType::MoveReverse)
    {
        targetSpeed = -0.3f;
        targetSteeringAngle = state.currentSteeringAngle;
        motionState = "后退";
    }
    else if (commandType == CommandType::MoveForward)
    {
        targetSpeed = 0.3f;
        targetSteeringAngle = state.currentSteeringAngle;
        motionState = "前进";
    }
    MotionDataPayload payload;
    payload.forwardSpeed = encodeSignedFixedPointValue(targetSpeed, 100.0f);
    payload.steeringAngle = encodeSteeringAngleToRaw(targetSteeringAngle, 10.0f);

    ROS_INFO_STREAM("Low-speed echo packet: speed=" << targetSpeed
                    << ", steering=" << targetSteeringAngle << " deg, motion=" << motionState);

    std::vector<uint8_t> dataBytes;
    dataBytes.push_back(static_cast<uint8_t>(payload.forwardSpeed & 0xFF));
    dataBytes.push_back(static_cast<uint8_t>((payload.forwardSpeed >> 8) & 0xFF));
    dataBytes.push_back(static_cast<uint8_t>(payload.steeringAngle & 0xFF));
    dataBytes.push_back(static_cast<uint8_t>((payload.steeringAngle >> 8) & 0xFF));

    const std::vector<uint8_t> packet = buildMotionPacket(0xEE, 0x01, dataBytes);
    ROS_INFO_STREAM("Sending low-speed echo packet: type=0x01, speed="
                    << decodeSignedFixedPointValue(payload.forwardSpeed, 100.0f)
                    << ", angle=" << decodeSteeringAngleFromRaw(payload.steeringAngle, 10.0f)
                    << "deg, crc=" << static_cast<int>(packet.back()));

    return sendPacket(virtualFd, packet);
}

}  // namespace vehicle_controller
