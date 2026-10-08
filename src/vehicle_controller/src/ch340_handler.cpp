#include "vehicle_controller/ch340_handler.h"

#include <ros/ros.h>

namespace vehicle_controller
{
ParsedCommand processCh340Frame(const std::array<uint8_t, 4>& frame, VehicleState& state)
{
    const auto parsed = parseFrame(frame);
    const std::string hex = bytesToHexString(frame);

    ROS_INFO_STREAM("CH340 received: " << hex);
    ROS_INFO_STREAM("CH340 semantic: " << parsed.detail
                    << " (" << commandTypeToString(parsed.type) << ")");

    if (parsed.type == CommandType::LowSpeedMode)
    {
        state.lowSpeedMode = true;
        state.currentMode = "低速模式";
        ROS_INFO("Mode switched to low speed mode");
    }
    else if (parsed.type == CommandType::IntermittentMode)
    {
        state.lowSpeedMode = false;
        state.currentMode = "间歇模式";
        ROS_INFO("Mode switched to intermittent mode");
    }
    else if (parsed.type == CommandType::IntelligentCruise)
    {
        state.lowSpeedMode = false;
        state.currentMode = "智能巡航";
        ROS_INFO("Mode switched to intelligent cruise mode");
    }

    if (parsed.requiresQrNavigation)
    {
        state.qrNavigationRequired = true;
        ROS_WARN_STREAM("命令 " << commandTypeToString(parsed.type) << " 需要识别二维码进行导航");
    }
    else
    {
        state.qrNavigationRequired = false;
    }

    return parsed;
}

}  // namespace vehicle_controller
