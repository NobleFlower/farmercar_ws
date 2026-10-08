#ifndef CH340_HANDLER_H
#define CH340_HANDLER_H

#include <array>
#include <cstdint>

#include "vehicle_controller_protocol.h"

namespace vehicle_controller
{
ParsedCommand processCh340Frame(const std::array<uint8_t, 4>& frame, VehicleState& state);
}  // namespace vehicle_controller

#endif
