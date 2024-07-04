#include "ForgeScan/Manager.hpp"
#include "ForgeScan/Simulation/GroundTruthScene.hpp"

#include "ForgeScan/Sensor/DepthImageProccessing.hpp"
#include "ForgeScan/Utilities/Timer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "forgescan_realsense/srv/camera_pose.hpp"

class ScanManager : public rclcpp::Node
{
    public:
        ScanManager()
        : Node("scan_manager")
        {
            auto scene = forge_scan::simulation::GroundTruthScene::create();
            manager = forge_scan::Manager::create(scene->grid_properties);
            reconstruction_service = this->create_service<std_srvs::srv::Empty>
                ("forgescan_realsense/reconstruction", std::bind(&ScanManager::run_reconstruction, this, std::placeholders::_1, std::placeholders::_2));
        }
    private:
        void run_reconstruction(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("image_client");
            rclcpp::Client<forgescan_realsense::srv::CameraPose>::SharedPtr client = node->create_client<forgescan_realsense::srv::CameraPose>("/camera/forgescan_realsense/camera_capture");
            auto pose_request = std::make_shared<forgescan_realsense::srv::CameraPose::Request>();

            manager->policyAdd("--set-active --type Axis --n-views 7 --n-repeat 3 --x -1.0 --y -1.0 --z -1.0 --seed 50 --uniform");
            while(!manager->policyIsComplete())
            {
                camera_pose = manager->policyGetView();

                Eigen::Matrix4f matrix_camera_pose = camera_pose.matrix();

                Eigen::Quaternionf quat(matrix_camera_pose.topLeftCorner<3,3>());
                
                auto camera_pose = geometry_msgs::msg::Pose();
                camera_pose.position.x = matrix_camera_pose(0,3);
                camera_pose.position.y = matrix_camera_pose(1,3);
                camera_pose.position.z = matrix_camera_pose(2,3);
                camera_pose.orientation.x = quat.x();
                camera_pose.orientation.y = quat.y();
                camera_pose.orientation.z = quat.z();
                camera_pose.orientation.w = quat.w();

                pose_request->pose = camera_pose;
                auto result_future = client->async_send_request(pose_request);

                if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) 
                {
                    auto result = result_future.get();
                } 
                else 
                {
                    RCLCPP_ERROR(node->get_logger(), "Failed to capture_image");
                }

                manager->policyAcceptView();
                
            }
        }
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reconstruction_service;
    std::shared_ptr<forge_scan::Manager> manager;
    forge_scan::Extrinsic camera_pose;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanManager>());
    rclcpp::shutdown();
    return 0;
}