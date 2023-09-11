#include "ForgeScan/Manager.hpp"
#include "ForgeScan/Simulation/Scene.hpp"
#include "ForgeScan/Utilities/Timer.hpp"


/// @brief Helper to create a camera model.
/// @param parser[out] ArgParser to use. On return this is set to the last arguments used for the camera.
/// @return Shared camera object.
inline std::shared_ptr<forge_scan::sensor::Camera> get_camera(forge_scan::utilities::ArgParser& parser)
{
    std::shared_ptr<forge_scan::sensor::Intrinsics> intr;
    while (true)
    {
        parser.getInput("\nPlease specify the camera intrinsic properties to use [-h for help]:");
        if (parser[0] != "-h")
        {
            intr = forge_scan::sensor::Intrinsics::create(parser);
            std::cout << "\nCamera model has the following intrinsics:"
                      << "\n\t" << *intr << std::endl;
            break;
        }
        std::cout << "\n" << forge_scan::sensor::Intrinsics::helpMessage() << std::endl;
    }
    return forge_scan::sensor::Camera::create(intr);
}



/// @brief Helper to add a polity to the Manager.
/// @param parser[out] ArgParser to use. On return this is set to the last arguments used to make a Metric.
/// @param manager[out] Manager to update.
inline void add_policy(forge_scan::utilities::ArgParser& parser,
                       std::shared_ptr<forge_scan::Manager>& manager)
{
    while (true)
    {
        parser.getInput("\nAdd a policy to the scene [-h for help]:");
        if (parser[0] == "-h")
        {
            if (parser[1] == "sphere")
            {
                std::cout << forge_scan::policies::Sphere::helpMessage() << std::endl;
            }
            else if (parser[1] == "axis")
            {
                std::cout << "\n" << forge_scan::policies::Axis::helpMessage() << std::endl;
            }
            else
            {
                std::cout << "\n" << forge_scan::policies::Policy::helpMessage() << std::endl;
            }
        }
        else
        {
            try
            {
                manager->policyAdd(parser);
                std::cout << "\nUsing the following policy:"
                          << "\n\t" << *manager->policyGetActive() << std::endl;
                return;
            }
            catch(const std::exception& e)
            {
                std::cerr << "Could not add policy: " << e.what() << '\n';
            }
        }
    }
}


/// @brief Helper to add Data Channels to the Manager.
/// @param parser[out] ArgParser to use. On return this is set to the last arguments used to make a Data Channel.
/// @param manager[out] Manager to update.
inline void add_data_channels(forge_scan::utilities::ArgParser& parser,
                              std::shared_ptr<forge_scan::Manager>& manager)
{
    while (true)
    {
        parser.getInput("\nAdd a data channels to the scene [-h for help or ENTER to finish]:");
        if (parser[0] == "-h")
        {
            if (parser[1] == "binary")
            {
                std::cout << forge_scan::data::Binary::helpMessage() << std::endl;
            }
            else if (parser[1] == "binarytsdf")
            {
                std::cout << "\n" << forge_scan::data::BinaryTSDF::helpMessage() << std::endl;
            }
            else if (parser[1] == "probability")
            {
                std::cout << "\n" << forge_scan::data::Probability::helpMessage() << std::endl;
            }
            else if (parser[1] == "tsdf")
            {
                std::cout << "\n" << forge_scan::data::TSDF::helpMessage() << std::endl;
            }
            else if (parser[1] == "updatecount")
            {
                std::cout << "\n" << forge_scan::data::UpdateCount::helpMessage() << std::endl;
            }
            else
            {
                std::cout << "\n" << forge_scan::data::VoxelGrid::helpMessage() << std::endl;
            }
        }
        else if (!parser.hasArgs())
        {
            return;
        }
        else
        {
            try
            {
                manager->reconstructionAddChannel(parser);
            }
            catch(const std::exception& e)
            {
                std::cerr << "Could not add data channel: " << e.what() << '\n';
            }
        }
    }
}


/// @brief Helper to add Metrics to the Manager.
/// @param parser[out] ArgParser to use. On return this is set to the last arguments used to make a Metric.
/// @param manager[out] Manager to update.
inline void add_metrics(forge_scan::utilities::ArgParser& parser,
                        std::shared_ptr<forge_scan::Manager>& manager)
{
    while (true)
    {
        parser.getInput("\nAdd metrics to the scene [-h for help or ENTER to finish]:");
        if (parser[0] == "-h")
        {
            /// TODO: Return an fill this in.
            std::cout << "TODO: Add metric help" << std::endl;
        }
        else if (!parser.hasArgs())
        {
            return;
        }
        else
        {
            try
            {
                manager->metricAdd(parser);
            }
            catch(const std::exception& e)
            {
                std::cerr << "Could not add metric: " << e.what() << '\n';
            }
        }
    }
}


int main()
{
    forge_scan::utilities::ArgParser parser;


    static const std::string default_scene_file_path = FORGE_SCAN_SHARE_DIR "/Examples/Scene.h5";
    static const std::string default_file_path = "ExperimentResults.h5";

    parser.getInput("Enter a file path for this experiment data:");
    std::filesystem::path fpath = parser.get<std::string>(0, default_file_path);


    // *********************************** SETUP EXPERIMENT ************************************ //


    // Load scene; set rejection rate.
    parser.getInput("Enter a file path to the ground truth scene to use:");
    std::filesystem::path scene_fpath = parser.get<std::string>(0, default_scene_file_path);
    auto scene = forge_scan::simulation::Scene::create();
    scene->load(scene_fpath);

    parser.getInput("Enter the view rejection rate to use (0 to 1):");
    const float reject_rate = std::clamp(parser.get<float>(0, 0.0f), 0.0f, 1.0f);


    // ***************************** Set up a Manager and a Policy ***************************** //

    auto manager = forge_scan::Manager::create(scene->grid_properties);

    // Get sensor model.
    auto camera = get_camera(parser);

    // Set the policy to test.
    add_policy(parser, manager);


    // *********************************** ADD DATA CHANNELS *********************************** //

    // Add data channels
    add_data_channels(parser, manager);


    // Add ground truth metrics
    add_metrics(parser, manager);
    auto bin_conf = forge_scan::metrics::OccupancyConfusion::create(manager->reconstruction,
                                                                    scene->getGroundTruthOccupancy(), 
                                                                    "binary");
    manager->metricAdd(bin_conf);


    // ******************************** GENERATE THEN SAVE DATA ******************************** //

    forge_scan::utilities::RandomSampler<float> rand_sample;
    forge_scan::utilities::Timer timer;
    forge_scan::PointMatrix sensed_points;

    timer.start();
    while (!manager->policyIsComplete())
    {
        forge_scan::Extrinsic camera_pose = manager->policyGetView();

        float val = rand_sample.uniform(); 
        if (val >= reject_rate)
        {
            manager->policyAcceptView();

            camera->setExtr(camera_pose);

            scene->image(camera);

            camera->getPointMatrix(sensed_points);
            manager->reconstructionUpdate(sensed_points, camera->getExtr());
        }
        else
        {
            manager->policyRejectView();
        }
    }
    timer.stop();

    std::cout << "Finished! Process took " << timer.elapsedSeconds() << " seconds." << std::endl;

    auto updated_fpath = manager->save(fpath);

    std::cout << "\nThe experimental data was successfully saved at:\n\t"
              << std::filesystem::absolute(fpath) << std::endl;

    return 0;
}