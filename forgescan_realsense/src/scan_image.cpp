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
#include <librealsense2/rs.hpp>

#include "forgescan_realsense/srv/camera_pose.hpp"
#include "forgescan_realsense/msg/eigen_vector.hpp"
#include "forgescan_realsense/srv/intrinsics.hpp"

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
        }
    private:
        void take_picture(const std::shared_ptr<forgescan_realsense::srv::CameraPose::Request> request,
                std::shared_ptr<forgescan_realsense::srv::CameraPose::Response> response)
        {
            std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("intrinsics_client");
            rclcpp::Client<forgescan_realsense::srv::Intrinsics>::SharedPtr client = node->create_client<forgescan_realsense::srv::Intrinsics>("/camera/forgescan_realsense/camera_intrinsics");

            auto empty_request = std::make_shared<forgescan_realsense::srv::Intrinsics::Request>();
            auto result = client->async_send_request(empty_request);

            if (result.valid() && rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) 
            {
                auto intrinsics_result = result.get();
                intr = forge_scan::sensor::Intrinsics::create(intrinsics_result->width, intrinsics_result->height, 
                                                            intrinsics_result->mindepth, intrinsics_result->maxdepth, 
                                                            intrinsics_result->fovx, intrinsics_result->fovy);
                RCLCPP_INFO(this->get_logger(), "Successfully retrieved intrinsics");
            } 
            else 
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to retrieve intrinsics values, using defaults");
                intr = forge_scan::sensor::Intrinsics::create();
            }
            camera = forge_scan::sensor::Camera::create(intr, 0.0, 100);
            geometry_msgs::msg::Pose pose = request->pose;
            Eigen::Quaternionf quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
            Eigen::Matrix4f transformation_matrix;
            transformation_matrix.setIdentity();
            transformation_matrix.block<3,3>(0,0) = quat.matrix();
            transformation_matrix.block<3,1>(0,3) = Eigen::Vector3f(pose.position.x, pose.position.y, pose.position.z);
            Eigen::Isometry3f camera_pose = Eigen::Isometry3f(transformation_matrix);
            camera->setExtr(camera_pose);

            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(camera_image, sensor_msgs::image_encodings::TYPE_16UC1);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            cv::Mat image = cv_ptr->image;

            cv::Mat depth_image_meters;

            image.convertTo(depth_image_meters, CV_32F, 10.0);

            for (int i = 0; i < depth_image_meters.rows; ++i) 
            {
                for(int j = 0; j < depth_image_meters.cols; j++)
                {
                    if(depth_image_meters.at<float>(i,j)<=1000)
                    {
                        depth_image_meters.at<float>(i,j) = 100000.0;
                    }
                    depth_image_meters.at<float>(i,j) = depth_image_meters.at<float>(i,j)/10000.0;
                    RCLCPP_INFO(this->get_logger(), "Depth value at (%d, %d): %f meters", i, j, depth_image_meters.at<float>(i, j));
                }
            }

            Eigen::MatrixXf eigen_image(depth_image_meters.rows, depth_image_meters.cols);
            for(int i = 0; i < depth_image_meters.rows; ++i)
            {
                for(int j = 0; j < depth_image_meters.cols; ++j)
                {
                    eigen_image(i, j) = depth_image_meters.at<float>(i,j);
                }
            }


            forge_scan::PointMatrix sensed_points;

            camera->getPointsFromImageAndIntrinsics(camera->getIntr(), eigen_image, sensed_points);

            response->eigenmatrix.resize(sensed_points.cols());
            response->length = sensed_points.cols();
            for(int i = 0; i<sensed_points.cols(); i++)
            {
                response->eigenmatrix[i].x = sensed_points.col(i).x();
                response->eigenmatrix[i].y = sensed_points.col(i).y();
                response->eigenmatrix[i].z = sensed_points.col(i).z();
            }
        }

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