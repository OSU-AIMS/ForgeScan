#include "ForgeScan/Manager.hpp"
#include "ForgeScan/Simulation/GroundTruthScene.hpp"

#include "ForgeScan/Sensor/DepthImageProccessing.hpp"
#include "ForgeScan/Utilities/Timer.hpp"

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "forgescan_realsense/srv/camera_pose.hpp"

class ScanImage : public rclcpp::Node
{
    public:
        ScanImage()
        : Node("scan_image")
        {
            realsense_subscriber = this->create_subscription<sensor_msgs::msg::Image>
            ("/camera/forgescan_realsense/camera_image", 10, std::bind(&ScanImage::realsense_image_callback, this, std::placeholders::_1));
            camera_capture_service = this->create_service<forgescan_realsense::srv::CameraPose>(
                "camera/forgescan_realsense/camera_capture", std::bind(&ScanImage::take_picture, this, std::placeholders::_1, std::placeholders::_2));
            auto intr = forge_scan::sensor::Intrinsics::create();
            camera = forge_scan::sensor::Camera::create(intr, 0.0, 100);
        }
    private:
        void take_picture(const std::shared_ptr<forgescan_realsense::srv::CameraPose::Request> request,
                std::shared_ptr<forgescan_realsense::srv::CameraPose::Response> response)
        {
            geometry_msgs::msg::Pose pose = request->pose;
            Eigen::Quaternionf quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
            Eigen::Matrix4f transformation_matrix;
            transformation_matrix.setIdentity();
            transformation_matrix.block<3,3>(0,0) = quat.matrix();
            transformation_matrix.block<3,1>(0,3) = Eigen::Vector3f(pose.position.x, pose.position.y, pose.position.z);
            Eigen::Isometry3f camera_pose = Eigen::Isometry3f(transformation_matrix);
            camera->setExtr(camera_pose);

            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(camera_image, sensor_msgs::image_encodings::TYPE_16UC1); //Might be different encoding, test to check.
            }
            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            cv::Mat cv_image = cv_ptr->image;
            Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> eigen_image; //Might be different datatype, float, look back through Scene to find out.
            cv::cv2eigen(cv_image, eigen_image);

            // Need to insert the image here before sensing points.

            forge_scan::PointMatrix sensed_points;
            camera->getPointMatrix(sensed_points);
        }

        void realsense_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr img)
        {
            camera_image = img;
        }
        sensor_msgs::msg::Image::ConstSharedPtr camera_image;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr realsense_subscriber;
        rclcpp::Service<forgescan_realsense::srv::CameraPose>::SharedPtr camera_capture_service;

        std::shared_ptr<forge_scan::sensor::Camera> camera;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanImage>());
    rclcpp::shutdown();
    return 0;
}