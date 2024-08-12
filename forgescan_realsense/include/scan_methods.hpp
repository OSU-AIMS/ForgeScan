#ifndef SCAN_METHODS_HPP
#define SCAN_METHODS_HPP

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "forgescan_realsense/srv/camera_pose.hpp"

#include "ForgeScan/Sensor/Camera.hpp"
#include "ForgeScan/Manager.hpp"

class ScanMethods {
public:
    Eigen::MatrixXf messageToEigen(const sensor_msgs::msg::Image::ConstSharedPtr& camera_image);

    void runTurnTableReconstruction(
        const std::shared_ptr<rclcpp::Node> node,
        const std::shared_ptr<forge_scan::sensor::Intrinsics> intr, 
        const std::shared_ptr<forge_scan::Manager> manager, 
        const rclcpp::Client<forgescan_realsense::srv::CameraPose>::SharedPtr image_client
    );
};
#endif 