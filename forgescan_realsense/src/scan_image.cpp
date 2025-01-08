#include "ForgeScan/Manager.hpp"
#include "ForgeScan/Simulation/GroundTruthScene.hpp"

#include "ForgeScan/Sensor/DepthImageProccessing.hpp"
#include "ForgeScan/Sensor/Intrinsics.hpp"
#include "ForgeScan/Utilities/Timer.hpp"

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "forgescan_realsense/srv/camera_pose.hpp"
#include "forgescan_realsense/msg/eigen_vector.hpp"
#include "forgescan_realsense/srv/intrinsics.hpp"

#include "scan_methods.hpp"

/* 
*  A ROS 2 Node to Handle the camera module of ForgeScan
*  Subscribes to realsense2 depth image topic
*  Creates a Camera_Capture service to return sensed points from image to scan_manager
*/
class ScanImage : public rclcpp::Node
{
    public:
        ScanImage()
        : Node("scan_image")
        {
            realsense_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/forgescan_realsense/camera_image", 10, 
                std::bind(&ScanImage::realsense_image_callback, this, std::placeholders::_1));
            camera_capture_service = this->create_service<forgescan_realsense::srv::CameraPose>(
                "camera/forgescan_realsense/camera_capture", 
                std::bind(&ScanImage::take_picture, this, 
                std::placeholders::_1, std::placeholders::_2));
        }
    private:
        /** 
        *  take_picture function takes in required pose for image and returns with an array of
        *  sensed points to describe depth image, as required by ForgeScan
        * 
        *  @param request: the request message that contains a geometry_msgs/Pose pose
        * 
        *  @param response: the response message that contains a EigenVector Array eigenmatrix, and an int length.
        *   where the EigenVector is a tuple of floats x, y, and z. 
        */
        void take_picture(const std::shared_ptr<forgescan_realsense::srv::CameraPose::Request> request,
                std::shared_ptr<forgescan_realsense::srv::CameraPose::Response> response)
        {
            ScanMethods scan_methods;
            Eigen::Matrix4f transformation_matrix;
            forge_scan::PointMatrix sensed_points;

            std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("intrinsics_client");
            rclcpp::Client<forgescan_realsense::srv::Intrinsics>::SharedPtr client = 
                node->create_client<forgescan_realsense::srv::Intrinsics>("/camera/forgescan_realsense/camera_intrinsics");
            auto intr = scan_methods.try_to_get_camera_intrinsics(node, client);
            camera = forge_scan::sensor::Camera::create(intr, 0.0, 100);

            auto camera_pose = scan_methods.rosPoseToIsometry(request->pose);
            camera->setExtr(camera_pose);

            Eigen::MatrixXf eigen_image = scan_methods.messageToEigen(camera_image);

            camera->getPointsFromImageAndIntrinsics(camera->getIntr(), eigen_image, sensed_points);

            response->eigenmatrix.resize(sensed_points.cols());
            for(int i = 0; i<sensed_points.cols(); i++)
            {
                response->eigenmatrix[i].x = sensed_points.col(i).x();
                response->eigenmatrix[i].y = sensed_points.col(i).y();
                response->eigenmatrix[i].z = sensed_points.col(i).z();
            }

            RCLCPP_INFO(this->get_logger(), "Successfully took picture #%ld", request->picture_number);
        }

        /**
        * Callback function for realsense depth image subscriber
        * 
        * @param img: Shard Pointer for the most recent camera image message to be used in all other functions within node.
        */
        void realsense_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr img)
        {
            camera_image = img;
        }

        sensor_msgs::msg::Image::ConstSharedPtr camera_image;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr realsense_subscriber;
        rclcpp::Service<forgescan_realsense::srv::CameraPose>::SharedPtr camera_capture_service;

        std::shared_ptr<forge_scan::sensor::Intrinsics> intr;
        std::shared_ptr<forge_scan::sensor::Camera> camera;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanImage>());
    rclcpp::shutdown();
    return 0;
}