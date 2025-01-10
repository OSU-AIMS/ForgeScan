#ifndef FORGESCAN_FORWARDMODEL_FORWARDMODELMANAGER_HPP
#define FORGESCAN_FORWARDMODEL_FORWARDMODELMANAGER_HPP

#include <iostream>
#include <fstream>

#include <string>

#include "ForgeScan/Manager.hpp"
#include "ForgeScan/Simulation/GroundTruthScene.hpp"

#include "ForgeScan/Sensor/DepthImageProccessing.hpp"
#include "ForgeScan/Utilities/Timer.hpp"
#include "ForgeScan/ForwardModel/WriteOut.hpp"
#include "ForgeScan/ForwardModel/MeshManipulator.hpp"

namespace forge_scan
{
    /**
     * @brief 
     * 
     */
    class ForwardModelManager
    {
        public:

        /**
         * @brief Creates a shared pointer to a ForwardModelManager object.
         * 
         * @return std::shared_ptr<ForwardModelManager>: a shared pointer to a ForwardModelManager object
         */
        static std::shared_ptr<ForwardModelManager> create(forge_scan::utilities::ArgParser parser)
        {
            return std::shared_ptr<ForwardModelManager>(new ForwardModelManager(parser));
        }

        /**
         * @brief Sets up the scene for the Forward Model
         * 
         */
        void setupScene()
        {
            std::string meshFileName = parser.get<std::string>("--load", "plain_football_box.obj");
            std::cout << "Setting up scene with " + meshFileName << std::endl;
            forge_scan::Extrinsic scene_lower_bound = forge_scan::Extrinsic::Identity();
            scene_lower_bound.translation() = forge_scan::Point( -1, -1, -1);
            scene = forge_scan::simulation::GroundTruthScene::create(scene_lower_bound);
            scene->add("--file " + meshFileName + " --x -0.5 --scale 5.0");
            auto updated_fpath = scene->save(scene_fpath);
            std::cout << "Scene contains: \n" << *scene << std::endl;
            std::cout << "Saved scene at " << updated_fpath << std::endl;

        }

        /**
         * @brief Sets up the Forward Model for the simulation
         * 
         */
        void setupForwardModel()
        {
            scene = forge_scan::simulation::GroundTruthScene::create();
            scene->load(scene_fpath);
            std::cout << "Scene contains: \n" << *scene << std::endl;
            intr = forge_scan::sensor::Intrinsics::create(parser);
            camera = forge_scan::sensor::Camera::create(intr, noise, 100);
            camera_pose = forge_scan::Extrinsic::Identity();
            manager = forge_scan::Manager::create(scene->grid_properties);
            write_out = forge_scan::WriteOut::create(img_fpath);
            mesh_manipulator = forge_scan::MeshManipulator::create(scene);
        }

        /**
         * @brief Gathers Ground Truth Data from the simulation module of ForgeScan
         * 
         */
        void gatherGroundTruthData()
        {
            forge_scan::utilities::Timer timer;
            timer.start();
            runGroundTruthCollection();
            timer.stop();
            std::cout << "Finished! Process took " << timer.elapsedSeconds() << " seconds." << std::endl;
        }

        /**
         * @brief Runs the Iterative Reconstruction for the Forward Model, currently set to 3 iterations
         * Will become dynamic in the future
         * 
         * @param desired_accuracy the desired accuracy of the final mesh in meters
         * @param accuracy_percentage the desired accuracy percentage of all nodes in the mesh
         */
        void runIterativeReconstruction(float desired_accuracy, float accuracy_percentage)
        {
            forge_scan::utilities::Timer timer;
            timer.start();
            runIterations(desired_accuracy, accuracy_percentage);
            timer.stop();
            std::cout << "Finished! Process took " << timer.elapsedSeconds() << " seconds." << std::endl;
        }

        /**
         * @brief The Stopping Criteria function for the MBIR process. Uses the desired accuracy to determine if a 
         * single node is within a stopping distance. Then uses the accuracy percentage to determine if the entire 
         * mesh is within the desired accuracy.
         * 
         * @param comparison_matrix: the matrix containing the x,y,z translations of each vertex
         * @param desired_accuracy: the desired accuracy of the final mesh in meters
         * @param accuracy_percentage: the desired accuracy percentage of all nodes in the mesh
         * @return true: reconstruction is done
         * @return false: reconstruction is not done
         */
        bool isReconstructionDone(
            Eigen::Matrix3Xf comparison_matrix,
            float desired_accuracy,
            float accuracy_percentage
        )
        {
            return mesh_manipulator->getTotalVeriticesOverThreshhold(comparison_matrix, desired_accuracy) > accuracy_percentage;
        }

        private:
        /**
         * @brief Private constructor to enforce use of shared pointers. Also sets up all fpaths
         *
         */
        explicit ForwardModelManager(forge_scan::utilities::ArgParser parser)
            : noise(parser.get<float>("--noise", 0.0f))
        {
            this->parser = parser;
            scene_fpath = parser.get<std::filesystem::path>("--scene", FORGE_SCAN_SHARE_DIR "/Examples/Scene.hdf5");
            save_fpath = parser.get<std::filesystem::path>("--save", FORGE_SCAN_SHARE_DIR "/Examples/Reconstruction.hdf5");
            mesh_fpath = parser.get<std::filesystem::path>("--save", FORGE_SCAN_SHARE_DIR "/Meshes/");
            img_fpath = std::string(FORGE_SCAN_SHARE_DIR) + "/Images/";
        }

        /**
         * @brief Helper function to run multiple iterations of the Model Based Reconstruction
         * 
         * @param desired_accuracy: the desired accuracy of the final mesh in meters
         * @param accuracy_percentage: the desired accuracy percentage of all nodes in the mesh
         */
        void runIterations(float desired_accuracy, float accuracy_percentage)
        {
            int iteration_count = 1;
            bool isReconstructionComplete = false;
            do
            {
                isReconstructionComplete = runIteration(iteration_count, desired_accuracy, accuracy_percentage);
                iteration_count++;
            } 
            while(!isReconstructionComplete);
            std::cout << "Reconstruction Complete!" << std::endl;
        }

        /**
         * @brief Runs a single iteration of the Model Based Reconstruction
         * 
         * @param iteration_count: the current iteration of the MBIR process
         * @param desired_accuracy: the desired accuracy of the final mesh in meters
         * @param accuracy_percentage: the desired accuracy percentage of all nodes in the mesh
         * @return true: if the reconstruction is done
         * @return false: if the reconstruction is not done
         */
        bool runIteration(int iteration_count, float desired_accuracy, float accuracy_percentage)
        {
            size_t n = 0;
            auto mesh = mesh_manipulator->extractMesh();
            int vertex_count = mesh.GetVertexPositions().GetLength();
            forge_scan::PointMatrix sensed_points;
            forge_scan::TriangleVector sensed_triangles;
            forge_scan::ComparisonMatrix comparison_matrix(4, vertex_count);
            comparison_matrix.setZero();
            manager->policyAdd("--set-active --type Sphere --n-views 10 --uniform --unordered --seed 50");
            while(!manager->policyIsComplete())
            {
                camera_pose = manager->policyGetView();
                manager->policyAcceptView();
                camera->setExtr(camera_pose);
                scene->image(camera, scene->grid_lower_bound);
                camera->getPointMatrix(sensed_points, sensed_triangles);
                comparison_matrix = mesh_manipulator->comparison(comparison_matrix, sensed_points, sensed_triangles, img_fpath, camera_pose, n);
                write_out->writeWithCameraData(sensed_points, sensed_triangles, n, iteration_count, camera_pose);
                write_out->writeZDiff(mesh_manipulator->getZDiff(sensed_points, img_fpath, n), n, iteration_count);
                std::cout << "Wrote out Point Cloud #" << n << std::endl;
                n++;
            }
            auto normalized_comparison_matrix = mesh_manipulator->normalizeComparisonMatrix(comparison_matrix,getStepSize(iteration_count));
            write_out->writeToCSV(normalized_comparison_matrix, mesh_manipulator->getVertexPositions(), iteration_count);
            auto terse_comparison_matrix = mesh_manipulator->matrix4XfToMatrix3Xf(normalized_comparison_matrix);
            mesh_manipulator->meshUpdate(terse_comparison_matrix);
            write_out->writeComparisonMatrix(normalized_comparison_matrix);
            std::string mesh_name = "iteration" + std::to_string(iteration_count) + ".obj";
            mesh_manipulator->writeMesh(mesh_fpath, mesh_name);
            loadMeshToScene(mesh_name);
            return isReconstructionDone(terse_comparison_matrix, desired_accuracy, accuracy_percentage);
        }

        /**
         * @brief Collects Ground Truth Data from Simulation Module of ForgeScan
         * 
         */
        void runGroundTruthCollection()
        {
            forge_scan::PointMatrix sensed_points;
            forge_scan::TriangleVector sensed_triangles;
            size_t n = 0;
            manager->policyAdd("--set-active --type Sphere --n-views 10 --uniform --unordered --seed 50");
            while(!manager->policyIsComplete())
            {
                camera_pose = manager->policyGetView();
                manager->policyAcceptView();
                camera->setExtr(camera_pose);
                scene->image(camera, scene->grid_lower_bound);
                camera->getPointMatrix(sensed_points, sensed_triangles);
                write_out->writePointCloud(sensed_points, n);
                std::cout << "Wrote out Point Cloud #" << n << std::endl;
                n++;
            }
        }

        /**
         * @brief Creates a new scene file with the new mesh. This should only be called after the mesh has been updated.
         * 
         * @param meshFileName the new mesh to be loaded in
         */
        void loadMeshToScene(std::string meshFileName)
        {
            std::cout << "Setting up scene with " + meshFileName << std::endl;
            forge_scan::Extrinsic scene_lower_bound = forge_scan::Extrinsic::Identity();
            scene_lower_bound.translation() = forge_scan::Point( -1, -1, -1);
            scene = forge_scan::simulation::GroundTruthScene::create(scene_lower_bound);
            scene->add("--file " + meshFileName);
            auto updated_fpath = scene->save(scene_fpath);
            std::cout << "Scene contains: \n" << *scene << std::endl;
            std::cout << "Saved scene at " << updated_fpath << std::endl;
        }
        
        /**
         * @brief Get the Step Size value based on iteration count
         * 
         * @param iteration: the current iteration of the MBIR process
         * @return float: the step size value to multiply against the comparison matrix
         */
        float getStepSize(int iteration)
        {
            float step_size = 1-exp(-iteration);
            return step_size;
        }

        // Global Variables

        const float noise;
        std::filesystem::path scene_fpath;
        std::filesystem::path save_fpath;
        std::filesystem::path mesh_fpath;
        std::filesystem::path img_fpath;
        forge_scan::Extrinsic camera_pose;
        forge_scan::utilities::ArgParser parser;

        // Shared Pointers to ForgeScan Objects

        std::shared_ptr<forge_scan::simulation::GroundTruthScene> scene;
        std::shared_ptr<forge_scan::sensor::Intrinsics> intr;
        std::shared_ptr<forge_scan::sensor::Camera> camera;
        std::shared_ptr<forge_scan::Manager> manager;
        std::shared_ptr<forge_scan::MeshManipulator> mesh_manipulator;
        std::shared_ptr<forge_scan::WriteOut> write_out;
    };
}

#endif