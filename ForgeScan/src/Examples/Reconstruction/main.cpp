#include "ForgeScan/Manager.hpp"
#include "ForgeScan/Simulation/GroundTruthScene.hpp"

#include "ForgeScan/Sensor/DepthImageProccessing.hpp"
#include "ForgeScan/Utilities/Timer.hpp"


int main(const int argc, const char **argv)
{
    forge_scan::utilities::ArgParser parser(argc, argv);
    const float reject_rate   = std::clamp(parser.get<float>("--reject", 0.0), 0.0f, 1.0f);
    const bool  show_im       = parser.has("--show-im");
    const bool  save_im       = parser.has("--save-im");
    const float noise         = parser.get<float>("--noise", 0.0f);

    std::filesystem::path scene_fpath = parser.get<std::filesystem::path>("--scene", FORGE_SCAN_SHARE_DIR "/Examples/Scene.hdf5");
    std::filesystem::path save_fpath  = parser.get<std::filesystem::path>("--save",  FORGE_SCAN_SHARE_DIR "/Examples/Reconstruction.hdf5");
    std::filesystem::path image_fpath = save_fpath;
    const std::string image_prefix    = save_fpath.filename().replace_extension("").string() + "View";

    // ************************************ Load the scene ************************************* //

    auto scene = forge_scan::simulation::GroundTruthScene::create();
    scene->load(scene_fpath);
    std::cout << "Scene contains: \n" << *scene << std::endl; 


    // ************************************ Create a camera ************************************ //

    auto intr   = forge_scan::sensor::Intrinsics::create(parser);
    auto camera = forge_scan::sensor::Camera::create(intr, noise, 100);
    auto camera_pose = forge_scan::Extrinsic::Identity();


    // ***************************** Set up a Manager and a Policy ***************************** //

    auto manager = forge_scan::Manager::create(scene->grid_properties);

    if (parser.has("--axis"))
    {
        std::cout << "using Axis policy..." << std::endl;
        manager->policyAdd("--set-active --type Axis --n-views 7 --n-repeat 3 --x -1.0 --y -1.0 --z -1.0 --seed 50 --uniform");
    }
    else if (parser.has("--occplane"))
    {
        std::cout << "using Occplane policy..." << std::endl;
        manager->policyAdd("--set-active --type Occplane --min-points 30 --eps 0.5");
    }
    else if (parser.has("--normal"))
    {
        std::cout << "using Normal policy..." << std::endl;
        manager->policyAdd("--set-active --type Normal --file share/pre.h5");
    }
    else
    {
        manager->policyAdd("--set-active --type Sphere --n-views 10 --uniform --unordered --seed 50");
    }

    manager->reconstructionAddChannel("--name tsdf           --type TSDF           --dtype double");
    manager->reconstructionAddChannel("--name avg_tsdf   --type TSDF   --average   --dtype float");
    manager->reconstructionAddChannel("--name min_tsdf   --type TSDF   --minimum   --dtype float");
    manager->reconstructionAddChannel("--name update         --type CountUpdates    --dtype uint");
    manager->reconstructionAddChannel("--name binary         --type Binary         --dtype uint");
    manager->reconstructionAddChannel("--name binary_tsdf    --type BinaryTSDF     --dtype uint");
    manager->reconstructionAddChannel("--name probability    --type Probability    --dtype float");
    manager->reconstructionAddChannel("--name views          --type CountViews     --dtype size_t");


    // ******* Set up an OccupancyConfusion Metric for the Scene, add it to the Manager ******** //


    auto bin_conf = forge_scan::metrics::OccupancyConfusion::create(manager->reconstruction,
                                                                    scene->getGroundTruthOccupancy(),
                                                                    "binary");
    manager->metricAdd(bin_conf);

    auto occ_conf = forge_scan::metrics::OccupancyConfusion::create(manager->reconstruction,
                                                                    scene->getGroundTruthOccupancy(),
                                                                    "probability");
    manager->metricAdd(occ_conf);

    auto tsdf_conf = forge_scan::metrics::OccupancyConfusion::create(manager->reconstruction,
                                                                    scene->getGroundTruthOccupancy(),
                                                                    "tsdf");
    manager->metricAdd(tsdf_conf);


    // ****************************** Collect and register images ****************************** //

    forge_scan::utilities::RandomSampler<float> rand_sample;
    forge_scan::utilities::Timer timer;

    forge_scan::PointMatrix sensed_points;
    size_t n = 0;

    timer.start();
    while (!manager->policyIsComplete())
    {
        camera_pose = manager->policyGetView();

        float val = rand_sample.uniform();
        if (val >= reject_rate)
        {
            manager->policyAcceptView();

            camera->setExtr(camera_pose);

            scene->image(camera, scene->grid_lower_bound);
            if (show_im)
            {
                forge_scan::sensor::DepthImageProcessing::imshow(camera, true);
            }
            if (save_im)
            {
                image_fpath.replace_filename(image_prefix + std::to_string(n) + ".jpg");
                forge_scan::sensor::DepthImageProcessing::imwrite(camera, image_fpath);
            }

            camera->getPointMatrix(sensed_points);
            manager->reconstructionUpdate(sensed_points, camera->getExtr());

            std::cout << "Added view: " << n++ << std::endl;
        }
        else
        {
            manager->policyRejectView();
            std::cout << "Rejected view: " << n++ << std::endl;
        }
    }
    timer.stop();

    auto updated_fpath = manager->save(save_fpath);

    std::cout << "Finished! Process took " << timer.elapsedSeconds() << " seconds." << std::endl;
    std::cout << "Saved scene at " << updated_fpath << std::endl;

    return 0;
}