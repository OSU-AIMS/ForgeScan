#include <iostream>
#include <fstream>

#include "ForgeScan/ForwardModel/ForwardModelManager.hpp"

int main(const int argc, const char **argv)
{
    forge_scan::utilities::ArgParser parser(argc, argv);
    auto manager = forge_scan::ForwardModelManager::create(parser);
    if(parser.has("--load"))
    {
        manager->setupScene();
    }
    if(parser.has("--groundTruth"))
    {
        std::cout << "Gathering Ground Truth Data" << std::endl;
        manager->setupForwardModel();
        manager->gatherGroundTruthData();
    }
    else
    {
        std::cout << "Running Iterative Reconstruction" << std::endl;
        manager->setupForwardModel();
        manager->runIterativeReconstruction(0.005, 0.95);
    }

    // CURRENT PROBLEM STANDS WITH MESHES NOT BEING PLACED CORRECTLY, NEED TO FIX THIS
}