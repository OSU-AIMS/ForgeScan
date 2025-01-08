#include "ForgeScan/Manager.hpp"
#include "ForgeScan/Simulation/GroundTruthScene.hpp"

#include "ForgeScan/Sensor/DepthImageProccessing.hpp"
#include "ForgeScan/Utilities/Timer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

class ScanSimulation : public rclcpp::Node
{
    public:
        ScanSimulation()
        : Node("scan_simulation")
        {
            auto intr = forge_scan::sensor::Intrinsics::create();
            auto camera = forge_scan::sensor::Camera::create(intr, 0.0, 100);
        }
    private:
        void take_picture()
        {
            //find out how to pass camera_pose to this as a service, find camera_pose's type and try to pass it over, should be simple after that.
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanSimulation>());
    rclcpp::shutdown();
    return 0;
}