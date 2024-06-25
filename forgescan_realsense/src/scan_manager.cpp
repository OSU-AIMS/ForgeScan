#include "ForgeScan/Manager.hpp"
#include "ForgeScan/Simulation/GroundTruthScene.hpp"

#include "ForgeScan/Sensor/DepthImageProccessing.hpp"
#include "ForgeScan/Utilities/Timer.hpp"


#include "rclcpp/rclcpp.hpp"

class ScanManager : public rclcpp::Node
{
    public:
        ScanManager()
        : Node("scan_manager")
        {
            auto manager = forge_scan::Manager::create(const std::shared_ptr<const Grid::Properties>& grid_properties = Grid::Properties::createConst());
            manager->policyAdd("--set-active --type Axis --n-views 7 --n-repeat 3 --x -1.0 --y -1.0 --z -1.0 --seed 50 --uniform");
        }
    private:
        void run_reconstruction()
        {
            while(!manager->policyIsComplete())
            {
                camera_pose = manager->policyGetView();
            }
        }

}