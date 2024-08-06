#include "ForgeScan/Manager.hpp"
#include "ForgeScan/Simulation/GroundTruthScene.hpp"

#include "ForgeScan/Sensor/DepthImageProccessing.hpp"
#include "ForgeScan/Utilities/Timer.hpp"
#include "ForgeScan/Sensor/Camera.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "forgescan_realsense/srv/camera_pose.hpp"
#include "forgescan_realsense/srv/intrinsics.hpp"

/*
*  A ROS 2 Node to handle the Scan Manager module of ForgeScan
*  Creates Reconstruction Service that runs an "axis" view selection reconstruction
*  with 7 views at 3 tilts 
*
*  Creates Two Clients for camera_capture service and camera_intrinsics service. 
*
*/
class ScanManager : public rclcpp::Node
{
    public:
        ScanManager()
        : Node("scan_manager")
        {
            auto scene = forge_scan::simulation::GroundTruthScene::create();
            manager = forge_scan::Manager::create(scene->grid_properties);
            reconstruction_service = this->create_service<std_srvs::srv::Empty>(
                "forgescan_realsense/reconstruction", 
                std::bind(&ScanManager::run_reconstruction, this, std::placeholders::_1, std::placeholders::_2));
        }
    private:
        /**
         * Main reconstruction service, runs a reconstruction through ForgeScan with a turntable view selection algorithm
         * This algorithm has 21 total views spread over 3 tilts.
         * 
         * Sends Service Request to Scan_Image to retrieve sensed_points matrix
         * 
         * @param request: Empty Service Request
         * 
         * @param response: Empty Service Response
         */
        void run_reconstruction(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            //Setting up ros clients/requests
            std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("image_client");
            rclcpp::Client<forgescan_realsense::srv::CameraPose>::SharedPtr image_client = 
                node->create_client<forgescan_realsense::srv::CameraPose>("/camera/forgescan_realsense/camera_capture");
            rclcpp::Client<forgescan_realsense::srv::Intrinsics>::SharedPtr intrinsics_client = 
                node->create_client<forgescan_realsense::srv::Intrinsics>("/camera/forgescan_realsense/camera_intrinsics");
            auto pose_request = std::make_shared<forgescan_realsense::srv::CameraPose::Request>();
            auto intrinsics_request = std::make_shared<forgescan_realsense::srv::Intrinsics::Request>();

            auto result = intrinsics_client->async_send_request(intrinsics_request);
            if (result.valid() && rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) 
            {
                auto intrinsics_result = result.get();
                intr = forge_scan::sensor::Intrinsics::create(
                    intrinsics_result->width, intrinsics_result->height, 
                    intrinsics_result->mindepth, intrinsics_result->maxdepth, 
                    intrinsics_result->fovx, intrinsics_result->fovy);
                RCLCPP_INFO(this->get_logger(), "Successfully retrieved intrinsics");
            } 
            else 
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to retrieve intrinsics values, using defaults");
                intr = forge_scan::sensor::Intrinsics::create();
            }

            //Setting up all local variables
            camera = forge_scan::sensor::Camera::create(intr, 0.0, 100);
            Eigen::Matrix3f K = Eigen::Matrix3f::Identity();
            forge_scan::PointMatrix sensed_points;

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

                //Add movement to move robot to position

                auto result_future = image_client->async_send_request(pose_request);

                if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) 
                {
                    auto result = result_future.get();
                    int len = result->length;
                    sensed_points.resize(3, len);
                    for(int i = 0; i<len; i++)
                    {
                        sensed_points(0, i) = result->eigenmatrix[i].x;
                        sensed_points(1, i) = result->eigenmatrix[i].y;
                        sensed_points(2, i) = result->eigenmatrix[i].z;
                    }
                } 
                else 
                {
                    RCLCPP_ERROR(node->get_logger(), "Failed to capture_image");
                }
                manager->reconstructionUpdate(sensed_points, camera->getExtr());

                manager->policyAcceptView();
            }
            RCLCPP_INFO(this->get_logger(), "Successfully finished Reconstruction");
        }

    std::shared_ptr<forge_scan::sensor::Intrinsics> intr;
    std::shared_ptr<forge_scan::sensor::Camera> camera;
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