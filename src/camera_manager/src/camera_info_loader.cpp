#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_info_loader");
    ros::NodeHandle nh("~");  // 使用私有命名空间以便传参
    ros::Publisher pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 10);

    std::string camera_name = "camera";
    std::string yaml_path;

    // 从参数读取YAML路径
    nh.param<std::string>("camera_info_url", yaml_path, "");

    camera_info_manager::CameraInfoManager cam_info_mgr(nh, camera_name, yaml_path);

    if (!cam_info_mgr.isCalibrated()) {
        ROS_WARN("Camera not calibrated or YAML file not found.");
    }

    ros::Rate rate(10);
    while (ros::ok()) {
        sensor_msgs::CameraInfo cam_info_msg = cam_info_mgr.getCameraInfo();
        cam_info_msg.header.stamp = ros::Time::now();
        cam_info_msg.header.frame_id = "camera_link";
        pub.publish(cam_info_msg);
        rate.sleep();
    }

    return 0;
}
