#ifndef VIRTUAL_SERIAL_HANDLER_H
#define VIRTUAL_SERIAL_HANDLER_H

#include <string>
#include <vector>

#include "vehicle_controller_protocol.h"

namespace vehicle_controller
{
bool processVirtualSerialPacket(const std::vector<uint8_t>& packet, VehicleState& state, const std::string& portName);
bool sendLowSpeedMotionControl(int virtualFd, const VehicleState& state, CommandType commandType);
}  // namespace vehicle_controller

#endif
