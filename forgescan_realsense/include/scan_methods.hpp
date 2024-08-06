#ifndef SCAN_METHODS_HPP
#define SCAN_METHODS_HPP

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <Eigen/Dense>

#include "ForgeScan/Sensor/Intrinsics.hpp"
#include "ForgeScan/Sensor/Camera.hpp"

class ScanMethods {
public:
    Eigen::MatrixXf messageToEigen(const sensor_msgs::msg::Image::ConstSharedPtr& camera_image);

    void runTurnTableReconstruction(const std::shared_ptr<forge_scan::sensor::Intrinsics> intr);
};
#endif 