#include <ros/ros.h>

#include <apriltag_ros/AprilTagDetectionArray.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <limits>


class AprilTagLineNavigation
{
public:

    AprilTagLineNavigation()
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        // ==============================
        // 读取参数
        // ==============================

        pnh.param("target_tag_id", target_tag_id_, 0);

        pnh.param("k_lateral", k_lateral_, 25.0);
        pnh.param("k_yaw", k_yaw_, 1.0);

        pnh.param("max_steering_angle",
                  max_steering_angle_,
                  50.0);

        pnh.param("tag_timeout",
                  tag_timeout_,
                  0.5);

        pnh.param("stop_distance",
                  stop_distance_,
                  0.5);

        pnh.param("print_frequency",
                  print_frequency_,
                  5.0);


        // ==============================
        // Subscriber
        // ==============================

        tag_sub_ = nh.subscribe(
            "/tag_detections",
            10,
            &AprilTagLineNavigation::tagCallback,
            this
        );


        // ==============================
        // Debug Publisher
        // ==============================

        lateral_error_pub_ =
            nh.advertise<std_msgs::Float64>(
                "/apriltag_navigation/lateral_error",
                10
            );

        yaw_error_pub_ =
            nh.advertise<std_msgs::Float64>(
                "/apriltag_navigation/yaw_error",
                10
            );

        steering_pub_ =
            nh.advertise<std_msgs::Float64>(
                "/apriltag_navigation/target_steering",
                10
            );

        tag_position_pub_ =
            nh.advertise<geometry_msgs::PointStamped>(
                "/apriltag_navigation/tag_position",
                10
            );


        last_tag_time_ = ros::Time(0.0);
        last_print_time_ = ros::Time(0.0);

        ROS_INFO("========================================");
        ROS_INFO(" AprilTag Line Navigation");
        ROS_INFO("========================================");

        ROS_INFO("Target Tag ID      : %d", target_tag_id_);
        ROS_INFO("K lateral           : %.3f", k_lateral_);
        ROS_INFO("K yaw               : %.3f", k_yaw_);
        ROS_INFO("Max steering        : %.3f deg",
                 max_steering_angle_);
        ROS_INFO("Tag timeout         : %.3f sec",
                 tag_timeout_);
        ROS_INFO("Stop distance       : %.3f m",
                 stop_distance_);

        ROS_INFO("----------------------------------------");
        ROS_INFO("SAFETY MODE:");
        ROS_INFO("NO vehicle command is sent.");
        ROS_INFO("Only theoretical steering is published.");
        ROS_INFO("========================================");
    }


private:

    // ============================================================
    // AprilTag callback
    // ============================================================

    void tagCallback(
        const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
    {
        bool target_found = false;

        double best_distance =
            std::numeric_limits<double>::max();

        const apriltag_ros::AprilTagDetection* best_detection =
            nullptr;


        // ========================================================
        // 遍历所有检测到的Tag
        // ========================================================

        for (const auto& detection : msg->detections)
        {
            if (detection.id.empty())
                continue;

            int tag_id = detection.id[0];

            if (tag_id != target_tag_id_)
                continue;


            const auto& position =
                detection.pose.pose.pose.position;


            double distance =
                std::sqrt(
                    position.x * position.x +
                    position.y * position.y +
                    position.z * position.z
                );


            if (distance < best_distance)
            {
                best_distance = distance;
                best_detection = &detection;
                target_found = true;
            }
        }


        // ========================================================
        // 没找到目标Tag
        // ========================================================

        if (!target_found)
        {
            return;
        }


        last_tag_time_ = ros::Time::now();


        // ========================================================
        // 获取位置
        // ========================================================

        const auto& position =
            best_detection->pose.pose.pose.position;


        double x = position.x;
        double y = position.y;
        double z = position.z;


        // ========================================================
        // 横向误差
        //
        // 当前先采用：
        //
        // e_y = -y
        //
        // 后续如果实际车辆转向方向相反，
        // 只需要修改这里的符号。
        // ========================================================

        double lateral_error = -y;


        // ========================================================
        // 四元数
        // ========================================================

        const auto& orientation =
            best_detection->pose.pose.pose.orientation;


        tf2::Quaternion q(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        );

        q.normalize();


        // ========================================================
        // 四元数 -> RPY
        // ========================================================

        double roll;
        double pitch;
        double yaw;

        tf2::Matrix3x3(q).getRPY(
            roll,
            pitch,
            yaw
        );


        // ========================================================
        // 当前将 yaw 作为航向误差
        //
        // 注意：
        // 这里后续需要根据你的相机安装方向和Tag姿态
        // 做一次实际验证。
        // ========================================================

        double yaw_error =
            yaw;


        // ========================================================
        // 控制律
        //
        // delta = Ky * ey + Kpsi * epsi
        // ========================================================

        double steering_angle =
            k_lateral_ * lateral_error +
            k_yaw_ * yaw_error * 180.0 / M_PI;


        // ========================================================
        // 限制最大转角
        // ========================================================

        if (steering_angle > max_steering_angle_)
        {
            steering_angle =
                max_steering_angle_;
        }

        if (steering_angle < -max_steering_angle_)
        {
            steering_angle =
                -max_steering_angle_;
        }


        // ========================================================
        // 距离过近
        // ========================================================

        bool too_close =
            best_distance < stop_distance_;


        if (too_close)
        {
            ROS_WARN_THROTTLE(
                1.0,
                "Tag too close: %.3f m",
                best_distance
            );
        }


        // ========================================================
        // 发布横向误差
        // ========================================================

        std_msgs::Float64 lateral_msg;

        lateral_msg.data =
            lateral_error;

        lateral_error_pub_.publish(
            lateral_msg
        );


        // ========================================================
        // 发布航向误差
        // 单位：deg
        // ========================================================

        std_msgs::Float64 yaw_msg;

        yaw_msg.data =
            yaw_error * 180.0 / M_PI;

        yaw_error_pub_.publish(
            yaw_msg
        );


        // ========================================================
        // 发布理论转向角
        // 单位：deg
        // ========================================================

        std_msgs::Float64 steering_msg;

        steering_msg.data =
            steering_angle;

        steering_pub_.publish(
            steering_msg
        );


        // ========================================================
        // 发布Tag位置
        // ========================================================

        geometry_msgs::PointStamped point_msg;

        point_msg.header =
            best_detection->pose.header;

        point_msg.point =
            position;

        tag_position_pub_.publish(
            point_msg
        );


        // ========================================================
        // 终端调试输出
        // ========================================================

        ros::Time now =
            ros::Time::now();

        if (
            (now - last_print_time_).toSec()
            >= 1.0 / print_frequency_
        )
        {
            ROS_INFO(
                "Tag[%d]  "
                "x=%+.3f  "
                "y=%+.3f  "
                "z=%+.3f  "
                "dist=%.3f  "
                "lateral=%+.3f  "
                "yaw=%+.2f deg  "
                "steering=%+.2f deg",
                target_tag_id_,
                x,
                y,
                z,
                best_distance,
                lateral_error,
                yaw_error * 180.0 / M_PI,
                steering_angle
            );

            last_print_time_ = now;
        }
    }


    // ============================================================
    // 成员变量
    // ============================================================

    ros::Subscriber tag_sub_;

    ros::Publisher lateral_error_pub_;
    ros::Publisher yaw_error_pub_;
    ros::Publisher steering_pub_;
    ros::Publisher tag_position_pub_;


    int target_tag_id_;

    double k_lateral_;
    double k_yaw_;

    double max_steering_angle_;

    double tag_timeout_;
    double stop_distance_;

    double print_frequency_;

    ros::Time last_tag_time_;
    ros::Time last_print_time_;
};


int main(int argc, char** argv)
{
    ros::init(
        argc,
        argv,
        "apriltag_line_nav"
    );

    AprilTagLineNavigation navigation;

    ros::spin();

    return 0;
}