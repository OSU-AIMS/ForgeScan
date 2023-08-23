#include "ForgeScan/Common/Grid.hpp"
#include "ForgeScan/Simulation/Scene.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


int main(const int argc, const char** argv)
{

    /************** Set up a scene **************/

    forge_scan::Extrinsic scene_lower_bound = forge_scan::Extrinsic::Identity();
    scene_lower_bound.translation() = forge_scan::Point( -1, -1, -1);

    auto scene = forge_scan::simulation::Scene::create(scene_lower_bound);

    scene->add("--name sphere1 --shape sphere --radius 0.25");
    scene->add("--name sphere2 --shape sphere --radius 0.15 --x 0.25 --y 0.25 --z 0.25");
    scene->add("--name box1    --shape box --l 1.25 --w 0.05 --h 0.75 --rx 6");

    /************** Calculate the ground truth **************/

    scene->calculateGroundTruthOccupancy();
    scene->calculateGroundTruthTSDF();

    /************** Write the scene to an HDF5 file **************/

    scene->save("GroundTruth.h5");

    scene->load("GroundTruth.h5");

    /************** Verify that we can re-load the HDF5 file **************/

    auto scene2 = forge_scan::simulation::Scene::create();
    scene2->load("GroundTruth.h5");

    return 0;
}