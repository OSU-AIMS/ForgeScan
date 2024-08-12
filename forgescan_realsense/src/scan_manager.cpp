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

#include "scan_methods.hpp"

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
            std::filesystem::path save_fpath  = "/home/bturner86239/forgescan_ws/src/reconstructions";

            manager->reconstructionAddChannel("--name tsdf           --type TSDF           --dtype double");
            manager->reconstructionAddChannel("--name avg_tsdf   --type TSDF   --average   --dtype float");
            manager->reconstructionAddChannel("--name min_tsdf   --type TSDF   --minimum   --dtype float");
            manager->reconstructionAddChannel("--name update         --type CountUpdates    --dtype uint");
            manager->reconstructionAddChannel("--name binary         --type Binary         --dtype uint");
            manager->reconstructionAddChannel("--name binary_tsdf    --type BinaryTSDF     --dtype uint");
            manager->reconstructionAddChannel("--name probability    --type Probability    --dtype float");
            manager->reconstructionAddChannel("--name views          --type CountViews     --dtype size_t");

            ScanMethods scan_methods;
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

            auto manager_node = std::make_shared<ScanManager>();
            scan_methods.runTurnTableReconstruction(node, intr, manager, image_client);

            auto updated_fpath = manager->save(save_fpath);
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