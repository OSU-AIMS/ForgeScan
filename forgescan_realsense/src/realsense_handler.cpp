//
// Created by bturner86239 on 6/17/24.
//

#include "chrono"
#include "memory"
#include "math.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/transform_stamped.h"
#include "geometry_msgs/msg/twist.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "forgescan_realsense/msg/intrinsics.hpp"
#include "forgescan_realsense/srv/intrinsics.hpp"
#include "forgescan_realsense/srv/to_transform.hpp"

using namespace std::chrono_literals;

/**
 * @brief 
 * 
 */
class RealsenseHandler : public rclcpp::Node
{
    public:
        RealsenseHandler()
        : Node("realsense_handler")
        {
            camera_intrinsics_subscriber = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera/camera/depth/camera_info", 10, std::bind(&RealsenseHandler::realsense_intrinsics_callback, this, std::placeholders::_1));
            intrinsics_service = this->create_service<forgescan_realsense::srv::Intrinsics>(
                "camera/forgescan_realsense/camera_intrinsics", std::bind(&RealsenseHandler::intrinsics_callback, this, std::placeholders::_1, std::placeholders::_2));
            tf_service = this->create_service<forgescan_realsense::srv::ToTransform>(
                "camera/forgescan_realsense/camera_transform", std::bind(&RealsenseHandler::get_transform, this, std::placeholders::_1, std::placeholders::_2));
            tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = 
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

    private:
        /**
         * @brief Gets data stored in Intrinsics Message variable as a service call. Message data is recieved from a topic but needed as a service.
         * 
         * @param request The request message which is empty
         * @param response The response message which contains all camera intrinsics
         */
        void intrinsics_callback(const std::shared_ptr<forgescan_realsense::srv::Intrinsics::Request>,
                std::shared_ptr<forgescan_realsense::srv::Intrinsics::Response> response)
        {
            response->width = message.width;
            response->height = message.height;
            response->mindepth = message.mindepth;
            response->maxdepth = message.maxdepth;
            response->fovx = message.fovx;
            response->fovy = message.fovy;
        }

        /**
         * @brief Sets Local Intrinsics Message Variable with data from camera/depth/camera_info topic
         * 
         * @param msg the camera intrinsics message data
         */
        void realsense_intrinsics_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
        {
            message.width = msg->width;
            message.height = msg->height;
            message.mindepth = 0.6; //Set due to this only being avaible through website and not message.
            message.maxdepth = 6.0; //Same as above
            message.fovx = (2 * atan ((message.width)/(2*msg->k[0]))) * 180 / M_PI;
            message.fovy = (2 * atan ((message.height)/(2*msg->k[4]))) * 180 / M_PI;
        }

        /**
         * @brief Gets the transform between two tf frames, defaults to "camera_link" and "object_ bounding_link"
         * 
         * @param request a request message with the "toFrame" and "fromFrame"
         * @param response 
         */
        void get_transform(const std::shared_ptr<forgescan_realsense::srv::ToTransform::Request> request,
                std::shared_ptr<forgescan_realsense::srv::ToTransform::Response> response)
        {
            std::string toFrameRel = request->toframe != "" ? request-> toframe : "camera_link";

            std::string fromFrameRel = request->fromframe != "" ? request->fromframe : "object_bounding_link";

            geometry_msgs::msg::TransformStamped t;

            try{
                t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);

            } catch (const tf2::TransformException & ex) {
                RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
                return;
            }
            response->transform = t.transform;
        }
    rclcpp::Publisher<forgescan_realsense::msg::Intrinsics>::SharedPtr intrinsics_publisher;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_intrinsics_subscriber;
    rclcpp::Service<forgescan_realsense::srv::ToTransform>::SharedPtr tf_service;
    rclcpp::Service<forgescan_realsense::srv::Intrinsics>::SharedPtr intrinsics_service;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    forgescan_realsense::msg::Intrinsics message;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealsenseHandler>());
    rclcpp::shutdown();
    return 0;
}