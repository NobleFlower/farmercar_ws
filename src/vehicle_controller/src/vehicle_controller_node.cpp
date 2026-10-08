#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/select.h>
#include <vector>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "vehicle_controller/ch340_handler.h"
#include "vehicle_controller/vehicle_controller_protocol.h"
#include "vehicle_controller/virtual_serial_handler.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_controller_node");
    ros::NodeHandle nh("~");

    std::string command_port = "/dev/ttyUSB0";
    std::string virtual_port = "/dev/ttyACM0";
    std::string speed_topic = "current_speed";
    int baudrate = B115200;

    nh.param("port", command_port, command_port);
    nh.param("virtual_port", virtual_port, virtual_port);
    nh.param("speed_topic", speed_topic, speed_topic);
    nh.param("baudrate", baudrate, baudrate);

    ROS_INFO_STREAM(
        "CH340 command port: " << command_port
        << ", virtual serial port: " << virtual_port
        << ", baudrate: " << baudrate
        << ", speed topic: " << speed_topic);

    ros::Publisher speed_pub =
        nh.advertise<std_msgs::Float32>(speed_topic, 10);

    // -------------------------------------------------------------------------
    // 打开 CH340 串口
    // -------------------------------------------------------------------------
    int command_fd =
        open(command_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if (command_fd < 0)
    {
        ROS_ERROR_STREAM(
            "Failed to open CH340 port "
            << command_port << ": " << strerror(errno));

        return 1;
    }

    // -------------------------------------------------------------------------
    // 打开虚拟串口
    // -------------------------------------------------------------------------
    int virtual_fd =
        open(virtual_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if (virtual_fd < 0)
    {
        ROS_ERROR_STREAM(
            "Failed to open virtual serial port "
            << virtual_port << ": " << strerror(errno));

        close(command_fd);
        return 1;
    }

    // -------------------------------------------------------------------------
    // 配置串口
    // -------------------------------------------------------------------------
    if (!vehicle_controller::configureSerialPort(command_fd, baudrate) ||
        !vehicle_controller::configureSerialPort(virtual_fd, baudrate))
    {
        close(command_fd);
        close(virtual_fd);
        return 1;
    }

    ROS_INFO("Default mode: low speed mode");

    // -------------------------------------------------------------------------
    // 初始化车辆状态
    // -------------------------------------------------------------------------
    vehicle_controller::VehicleState state;

    state.lowSpeedMode = true;
    state.currentLowSpeedValue = 0.0f;
    state.currentMotion = "停车";
    state.currentMode = "低速模式";

    // -------------------------------------------------------------------------
    // CH340：固定 4 字节命令帧
    // -------------------------------------------------------------------------
    std::array<uint8_t, 4> frame{};
    size_t frame_index = 0;

    // -------------------------------------------------------------------------
    // 虚拟串口：EF 开头、8 字节的数据包缓存
    // -------------------------------------------------------------------------
    std::vector<uint8_t> ef_packet_buffer;

    // -------------------------------------------------------------------------
    // 主循环
    // -------------------------------------------------------------------------
    while (ros::ok())
    {
        fd_set read_fds;

        FD_ZERO(&read_fds);
        FD_SET(command_fd, &read_fds);
        FD_SET(virtual_fd, &read_fds);

        const int max_fd =
            command_fd > virtual_fd ? command_fd : virtual_fd;

        const int select_result =
            select(max_fd + 1, &read_fds, nullptr, nullptr, nullptr);

        if (select_result < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }

            ROS_ERROR_STREAM(
                "Serial select error: " << strerror(errno));

            break;
        }

        // =====================================================================
        // 1. CH340 串口
        //
        // CH340 是“命令输入端”：
        //
        // CH340
        //   ↓
        // processCh340Frame()
        //   ↓
        // 解析命令
        //   ↓
        // sendLowSpeedMotionControl()
        //   ↓
        // 虚拟串口
        //   ↓
        // 下位机
        //
        // 注意：
        // 这里是运动控制帧的唯一发送入口。
        // =====================================================================
        if (FD_ISSET(command_fd, &read_fds))
        {
            uint8_t byte = 0;

            const ssize_t ret =
                read(command_fd, &byte, 1);

            if (ret == 1)
            {
                frame[frame_index++] = byte;

                // 收满 4 字节
                if (frame_index == frame.size())
                {
                    const auto parsed =
                        vehicle_controller::processCh340Frame(
                            frame,
                            state);

                    // ---------------------------------------------------------
                    // 低速模式下，只处理前进 / 停车 / 后退
                    // ---------------------------------------------------------
                    if (state.lowSpeedMode &&
                        (parsed.type ==
                             vehicle_controller::CommandType::MoveForward ||
                         parsed.type ==
                             vehicle_controller::CommandType::MoveStop ||
                         parsed.type ==
                             vehicle_controller::CommandType::MoveReverse))
                    {
                        float commandSpeed = 0.0f;

                        if (parsed.type ==
                            vehicle_controller::CommandType::MoveReverse)
                        {
                            commandSpeed = -0.3f;
                        }
                        else if (parsed.type ==
                                 vehicle_controller::CommandType::MoveForward)
                        {
                            commandSpeed = 0.3f;
                        }
                        else
                        {
                            commandSpeed = 0.0f;
                        }

                        // -----------------------------------------------------
                        // 发布 ROS 当前命令速度
                        // -----------------------------------------------------
                        std_msgs::Float32 speed_msg;
                        speed_msg.data = commandSpeed;
                        speed_pub.publish(speed_msg);

                        // -----------------------------------------------------
                        // 更新当前运动状态
                        // -----------------------------------------------------
                        if (parsed.type ==
                            vehicle_controller::CommandType::MoveStop)
                        {
                            state.currentMotion = "停车";
                        }
                        else if (parsed.type ==
                                 vehicle_controller::CommandType::MoveReverse)
                        {
                            state.currentMotion = "后退";
                        }
                        else
                        {
                            state.currentMotion = "前进";
                        }

                        // -----------------------------------------------------
                        // 向下位机发送运动控制帧
                        //
                        // 这是唯一一次发送。
                        // -----------------------------------------------------
                        ROS_INFO_STREAM(
                            "CH340 command -> sending motion command: "
                            << state.currentMotion);

                        vehicle_controller::sendLowSpeedMotionControl(
                            virtual_fd,
                            state,
                            parsed.type);
                    }

                    // ---------------------------------------------------------
                    // 当前 4 字节帧处理完成，准备下一帧
                    // ---------------------------------------------------------
                    frame_index = 0;
                    frame.fill(0);
                }
            }
            else if (ret < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }

                ROS_ERROR_STREAM(
                    "CH340 serial read error: "
                    << strerror(errno));

                break;
            }
        }

        // =====================================================================
        // 2. 虚拟串口
        //
        // 虚拟串口是“下位机反馈输入端”：
        //
        // 下位机
        //   ↓
        // 虚拟串口
        //   ↓
        // processVirtualSerialPacket()
        //   ↓
        // 更新 VehicleState
        //   ↓
        // 发布 ROS
        //
        // 特别注意：
        // 这里绝对不再调用 sendLowSpeedMotionControl()。
        //
        // 否则会形成：
        //
        // 上位机发送
        //   ↓
        // 下位机
        //   ↓
        // 下位机反馈
        //   ↓
        // 上位机接收
        //   ↓
        // 再次发送
        //
        // 从而导致下位机可能收到两次运动命令。
        // =====================================================================
        if (FD_ISSET(virtual_fd, &read_fds))
        {
            uint8_t byte = 0;

            const ssize_t ret =
                read(virtual_fd, &byte, 1);

            if (ret == 1)
            {
                // -------------------------------------------------------------
                // 只有检测到 0xEF 帧头之后才开始缓存
                // -------------------------------------------------------------
                if (!ef_packet_buffer.empty() || byte == 0xEF)
                {
                    ef_packet_buffer.push_back(byte);

                    // ---------------------------------------------------------
                    // 收满 8 字节
                    // ---------------------------------------------------------
                    if (ef_packet_buffer.size() == 8)
                    {
                        if (vehicle_controller::processVirtualSerialPacket(
                                ef_packet_buffer,
                                state,
                                virtual_port))
                        {
                            // -------------------------------------------------
                            // 下位机反馈有效
                            // 更新 ROS 当前速度
                            // -------------------------------------------------
                            std_msgs::Float32 msg;

                            msg.data =
                                state.currentLowSpeedValue;

                            speed_pub.publish(msg);

                            // -------------------------------------------------
                            // 重要：
                            // 这里只接收和处理下位机反馈。
                            //
                            // 不再调用：
                            //
                            // sendLowSpeedMotionControl()
                            //
                            // 防止产生二次发送。
                            // -------------------------------------------------

                            ef_packet_buffer.clear();
                        }
                        else
                        {
                            ROS_WARN(
                                "Invalid virtual serial packet, "
                                "searching for the next frame header");

                            // -------------------------------------------------
                            // 当前数据包非法，寻找下一个 0xEF
                            // -------------------------------------------------
                            size_t next_header_index = 1;

                            while (
                                next_header_index <
                                    ef_packet_buffer.size() &&
                                ef_packet_buffer[next_header_index] != 0xEF)
                            {
                                ++next_header_index;
                            }

                            if (next_header_index <
                                ef_packet_buffer.size())
                            {
                                ef_packet_buffer.erase(
                                    ef_packet_buffer.begin(),
                                    ef_packet_buffer.begin() +
                                        next_header_index);
                            }
                            else
                            {
                                ef_packet_buffer.clear();
                            }
                        }
                    }
                }
            }
            else if (ret < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }

                ROS_ERROR_STREAM(
                    "Virtual serial read error: "
                    << strerror(errno));

                break;
            }
        }

        // ---------------------------------------------------------------------
        // ROS 回调
        // ---------------------------------------------------------------------
        ros::spinOnce();
    }

    // -------------------------------------------------------------------------
    // 关闭串口
    // -------------------------------------------------------------------------
    close(command_fd);
    close(virtual_fd);

    return 0;
}
