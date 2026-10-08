#ifndef VEHICLE_CONTROLLER_PROTOCOL_H
#define VEHICLE_CONTROLLER_PROTOCOL_H

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace vehicle_controller
{
enum class CommandType
{
    Unknown,
    MoveForward,
    MoveReverse,
    MoveStop,
    Unload,
    Reset,
    LowSpeedMode,
    IntermittentMode,
    IntelligentCruise,
    GoAisle
};

struct ParsedCommand
{
    CommandType type = CommandType::Unknown;
    std::string detail = "无法识别的帧";
    bool requiresQrNavigation = false;
};

struct VehicleState
{
    bool lowSpeedMode = true;
    bool qrNavigationRequired = false;
    float currentLowSpeedValue = 0.0f;
    float currentSteeringAngle = 90.0f;
    std::string currentMotion = "停车";
    std::string currentMode = "低速模式";
    int targetAisle = 0;
};

struct MotionDataPayload
{
    int16_t forwardSpeed = 0;
    uint16_t steeringAngle = 900;
};

inline std::string commandTypeToString(CommandType type)
{
    switch (type)
    {
        case CommandType::MoveForward:
            return "前进控制";
        case CommandType::MoveReverse:
            return "后退控制";
        case CommandType::MoveStop:
            return "停车控制";
        case CommandType::Unload:
            return "卸货控制";
        case CommandType::Reset:
            return "复位控制";
        case CommandType::LowSpeedMode:
            return "低速模式";
        case CommandType::IntermittentMode:
            return "间歇模式";
        case CommandType::IntelligentCruise:
            return "智能巡航";
        case CommandType::GoAisle:
            return "过道导航";
        case CommandType::Unknown:
        default:
            return "未知命令";
    }
}

inline std::string bytesToHexString(const std::array<uint8_t, 4>& frame)
{
    std::ostringstream oss;
    oss << std::hex << std::uppercase;
    for (size_t i = 0; i < frame.size(); ++i)
    {
        if (i != 0)
        {
            oss << " ";
        }
        oss << std::setw(2) << std::setfill('0') << static_cast<int>(frame[i]);
    }
    return oss.str();
}

template <typename T>
inline T clampValue(const T& value, const T& low, const T& high)
{
    return value < low ? low : (value > high ? high : value);
}

inline float decodeSignedFixedPointValue(int16_t rawValue, float scale = 100.0f)
{
    return static_cast<float>(rawValue) / scale;
}

inline float decodeSteeringAngleFromRaw(uint16_t rawAngle, float scale = 10.0f)
{
    return static_cast<float>(rawAngle) / scale;
}

inline int16_t encodeSignedFixedPointValue(float value, float scale = 100.0f)
{
    const float clamped = clampValue(value, -327.68f, 327.67f);
    return static_cast<int16_t>(std::lround(clamped * scale));
}

inline uint16_t encodeSteeringAngleToRaw(float angleDeg, float scale = 10.0f)
{
    const float clamped = clampValue(angleDeg, 0.0f, 180.0f);
    return static_cast<uint16_t>(std::lround(clamped * scale));
}

inline uint8_t computeCrc(const std::vector<uint8_t>& bytes)
{
    uint8_t crc = 0;
    for (uint8_t byte : bytes)
    {
        crc = static_cast<uint8_t>(crc + byte);
    }
    return crc;
}

inline bool parseVirtualSpeedPacket(const std::vector<uint8_t>& packet, float& speedValue, float& steeringAngleDeg)
{
    if (packet.size() != 8)
    {
        return false;
    }

    if (packet[0] != 0xEF)
    {
        return false;
    }

    const uint8_t type = packet[1];
    const uint8_t len = packet[2];
    const uint8_t crc = packet.back();
    if (type != 0x01 || len != 0x04)
    {
        return false;
    }

    std::vector<uint8_t> crcPayload(packet.begin(), packet.end() - 1);
    if (computeCrc(crcPayload) != crc)
    {
        return false;
    }

    const int16_t rawSpeed = static_cast<int16_t>(
        static_cast<uint16_t>(packet[3]) | (static_cast<uint16_t>(packet[4]) << 8U));
    const uint16_t rawSteering = static_cast<uint16_t>(packet[5]) |
                                (static_cast<uint16_t>(packet[6]) << 8U);

    speedValue = static_cast<float>(rawSpeed) / 100.0f;
    steeringAngleDeg = decodeSteeringAngleFromRaw(rawSteering, 10.0f);
    return true;
}

inline ParsedCommand parseFrame(const std::array<uint8_t, 4>& frame)
{
    ParsedCommand cmd;

    if (frame[0] == 0xAB && frame[1] == 0xAB && frame[2] == 0xAB)
    {
        switch (frame[3])
        {
            case 0x01:
                cmd.type = CommandType::MoveForward;
                cmd.detail = "前进";
                break;
            case 0x10:
                cmd.type = CommandType::MoveReverse;
                cmd.detail = "后退";
                break;
            case 0x00:
                cmd.type = CommandType::MoveStop;
                cmd.detail = "停车";
                break;
            default:
                cmd.type = CommandType::Unknown;
                cmd.detail = "未知运动控制";
                break;
        }
    }
    else if (frame[0] == 0xCD && frame[1] == 0xCD && frame[2] == 0xCD)
    {
        switch (frame[3])
        {
            case 0x01:
                cmd.type = CommandType::Unload;
                cmd.detail = "卸货";
                cmd.requiresQrNavigation = true;
                break;
            case 0x10:
                cmd.type = CommandType::Reset;
                cmd.detail = "复位";
                cmd.requiresQrNavigation = true;
                break;
            default:
                cmd.type = CommandType::Unknown;
                cmd.detail = "未知卸货/复位指令";
                cmd.requiresQrNavigation = true;
                break;
        }
    }
    else if (frame[0] == 0xDE && frame[1] == 0xDE && frame[2] == 0xDE)
    {
        switch (frame[3])
        {
            case 0x00:
                cmd.type = CommandType::LowSpeedMode;
                cmd.detail = "低速模式";
                break;
            case 0x01:
                cmd.type = CommandType::IntermittentMode;
                cmd.detail = "间歇模式";
                break;
            case 0x02:
                cmd.type = CommandType::IntelligentCruise;
                cmd.detail = "智能巡航";
                cmd.requiresQrNavigation = true;
                break;
            default:
                cmd.type = CommandType::Unknown;
                cmd.detail = "未知模式";
                cmd.requiresQrNavigation = true;
                break;
        }
    }
    else if (frame[0] == 0xAD && frame[1] == 0xAD && frame[2] == 0xAD)
    {
        if (frame[3] >= 0x01 && frame[3] <= 0x09)
        {
            cmd.type = CommandType::GoAisle;
            cmd.detail = "前往第 " + std::to_string(frame[3]) + " 过道";
            cmd.requiresQrNavigation = true;
        }
        else
        {
            cmd.type = CommandType::Unknown;
            cmd.detail = "非法过道编号";
            cmd.requiresQrNavigation = true;
        }
    }
    else
    {
        cmd.type = CommandType::Unknown;
        cmd.detail = "无法识别的帧";
    }

    return cmd;
}

inline std::vector<uint8_t> buildMotionPacket(uint8_t header, uint8_t type, const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> packet;
    packet.push_back(header);
    packet.push_back(type);
    packet.push_back(static_cast<uint8_t>(payload.size()));
    packet.insert(packet.end(), payload.begin(), payload.end());
    packet.push_back(computeCrc(packet));
    return packet;
}

inline bool sendPacket(int fd, const std::vector<uint8_t>& packet)
{
    if (fd < 0)
    {
        return false;
    }

    const ssize_t written = write(fd, packet.data(), packet.size());
    if (written < 0)
    {
        ROS_ERROR_STREAM("Failed to send packet: " << strerror(errno));
        return false;
    }

    if (static_cast<size_t>(written) != packet.size())
    {
        ROS_WARN_STREAM("Partial packet written: " << written << "/" << packet.size());
        return false;
    }

    return true;
}

inline bool configureSerialPort(int fd, int baudrate)
{
    termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        ROS_ERROR_STREAM("tcgetattr failed: " << strerror(errno));
        return false;
    }

    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        ROS_ERROR_STREAM("tcsetattr failed: " << strerror(errno));
        return false;
    }

    return true;
}

}  // namespace vehicle_controller

#endif
