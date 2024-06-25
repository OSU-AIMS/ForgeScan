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

using namespace std::chrono_literals;

class RealsenseHandler : public rclcpp::Node
{
    public:
        RealsenseHandler()
        : Node("realsense_handler")
        {
            intrinsics_publisher = this->create_publisher<forgescan_realsense::msg::Intrinsics>("camera/forgescan_realsense/camera_intrinsics", 10);
            camera_intrinsics_subscriber = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera/camera/depth/camera_info", 10, std::bind(&RealsenseHandler::intrinsics_callback, this, std::placeholders::_1));

            tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = 
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            timer_ = this->create_wall_timer(
                1s, std::bind(&RealsenseHandler::get_transform, this));
            tf_publisher = this-> create_publisher<geometry_msgs::msg::TransformStamped>("camera/forgescan_realsense/camera_transform", 1);
        }

    private:
        void intrinsics_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
        {
            auto message = forgescan_realsense::msg::Intrinsics();
            message.width = msg->width;
            message.height = msg->height;
            message.mindepth = 0.6; //Set due to this only being avaible through website and not message.
            message.maxdepth = 6.0; //Same as above
            message.fovx = (2 * atan ((message.width)/(2*msg->k[0]))) * 180 / M_PI;
            message.fovy = (2 * atan ((message.height)/(2*msg->k[4]))) * 180 / M_PI;
            this->intrinsics_publisher->publish(message);
        }

        void get_transform()
        {
            std::string fromFrameRel = "object_bounding_link";
            std::string toFrameRel = "camera_link";

            geometry_msgs::msg::TransformStamped t;

            try{
                t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);

            } catch (const tf2::TransformException & ex) {
                RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
                return;
            }
            tf_publisher->publish(t);

        }
    rclcpp::Publisher<forgescan_realsense::msg::Intrinsics>::SharedPtr intrinsics_publisher;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_intrinsics_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_publisher;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealsenseHandler>());
    rclcpp::shutdown();
    return 0;
}