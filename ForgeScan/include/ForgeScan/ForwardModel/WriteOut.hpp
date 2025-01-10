#ifndef FORGESCAN_FORWARDMODEL_WRITEOUT_HPP
#define FORGESCAN_FORWARDMODEL_WRITEOUT_HPP

#include <iostream>
#include <fstream>
#include <filesystem>

#include "ForgeScan/Common/Types.hpp"

namespace forge_scan
{
    /**
     * @brief The WriteOut Class, designed to assist in writing out any point cloud data to xyz file formats
     *
     */
    class WriteOut
    {
    public:
        /**
         * @brief Creates a shared pointer to a WriteOut object.
         *
         * @param spath the save filepath
         * @return std::shared_ptr<WriteOut>: a shared pointed to a WriteOut object.
         */
        static std::shared_ptr<WriteOut> create(std::filesystem::path spath)
        {
            return std::shared_ptr<WriteOut>(new WriteOut(spath));
        }

        /**
         * @brief Writes out to a .xyz file with camera pose, point data, and triangle data.
         *
         * @param sensed_points contains the x,y,z value of a given pointcloud
         * @param sensed_triangles contains corresponding triangles to sensed_point data
         * @param img_count the # image to be written out
         * @param camera_pose the current camera position as an Isometry3f
         * @return true: if writeout was successful
         * @return false: if writeout wasn't successful
         */
        bool writeWithCameraData(
            forge_scan::PointMatrix sensed_points,
            forge_scan::TriangleVector sensed_triangles,
            int img_count,
            int iteration_count,
            Eigen::Isometry3f camera_pose)
        {
            int success = 0;
            std::ofstream outFile(std::string(save_path) + "stl_pointcloud_with_intrinsics" + std::to_string(iteration_count) + "_" + std::to_string(img_count) + ".xyz");
            writeCameraData(camera_pose, outFile);
            outFile << "----------------------------------------" << std::endl;
            writePointCloudWithTriangleData(outFile, sensed_points, sensed_triangles);
            return success;
        }

        /**
         * @brief Writes out to a .xyz file with point data and triangle data.
         *
         * @param sensed_points contains the x,y,z value of a given pointcloud
         * @param sensed_triangles contains corresponding triangles to sensed_point data
         * @param img_count the # image to be written out
         * @return true: if writeout was successful
         * @return false: if writeout wasn't successful
         */
        bool writeWithoutCameraData(
            forge_scan::PointMatrix sensed_points,
            forge_scan::TriangleVector sensed_triangles,
            int img_count)
        {
            int success = 0;
            std::ofstream outFile(std::string(save_path) + "stl_pointcloud" + std::to_string(img_count) + ".xyz");
            writePointCloudWithTriangleData(outFile, sensed_points, sensed_triangles);
            return success;
        }

        /**
         * @brief Writes out to a .xyz file with point data. (PointCloud)
         *
         * @param sensed_points contains the x,y,z value of a given pointcloud
         * @param img_count the # image to be written out
         * @return true: if writeout was successful
         * @return false: if writeout wasn't successful
         */
        bool writePointCloud(forge_scan::PointMatrix sensed_points, int img_count)
        {
            int success = 0;
            std::ofstream outFile(std::string(save_path) + "groundtruth_pointcloud" + std::to_string(img_count) + ".xyz");
            printPointCloud(outFile, sensed_points);
            return success;
        }

        /**
         * @brief Writing out the zDifference vector to a .txt file
         * 
         * @param zDiff the zDifference vector
         * @param img_count the iteration for the given zDifference
         * @return true: if the writeout was successful 
         * @return false: if the writeout was unsuccessful
         */
        bool writeZDiff(Eigen::VectorXf zDiff, int img_count, int iteration_count)
        {
            int success = 0;
            std::ofstream outFile(std::string(save_path) + "zDifference_" + std::to_string(iteration_count) + "_" + std::to_string(img_count) + ".txt");
            printZDiff(outFile, zDiff);
            return success;
        }

        /**
         * @brief Write out the comparison matrix to a .txt file
         * 
         * @param comparison the comparison matrix (x,y,z and amount of times the node has been hit)
         * @return true: if the writeout was successful
         * @return false: if the writeout was unsuccessful
         */
        bool writeComparisonMatrix(Eigen::Matrix4Xf comparison)
        {
            int success = 0;
            std::ofstream outFile(std::string(save_path) + "Comparison_Matrix.txt");
            printComparisonMatrix(outFile, comparison);
            return success;
        }

        /**
         * @brief Writing out to a CSV file the original x,y,z coordinates of a node, then the delta x,y,z, and then the amount of times that node has been told to move.
         * 
         * @param comparison the comparison matrix that holds delta x, delta y, delta z, and the amount of times the node has been seen
         * @param vertex_positions the original positions of each node (x,y,z)
         * @param iteration the iteration number
         * @return true: if the writeout was successful
         * @return false: if the writeout was unsuccesful
         */
        bool writeToCSV(
            Eigen::Matrix4Xf comparison,
            Eigen::Matrix3Xf vertex_positions,
            int iteration
        )
        {
            int success = 0;
            std::ofstream outFile(std::string(save_path) + "Vertex_Vector_Data_" + std::to_string(iteration) + ".csv");
            for(int i = 0; i<vertex_positions.cols(); i++)
            {
                outFile 
                    << vertex_positions.col(i).x() << "," 
                    << vertex_positions.col(i).y() << "," 
                    << vertex_positions.col(i).z() << "," 
                    << comparison.col(i).x() << "," 
                    << comparison.col(i).y() << "," 
                    << comparison.col(i).z() << "," 
                    << comparison.col(i).w() << std::endl;
            }
            return success;
        }

    private:
        /**
         * @brief Private constructor to enforce use of shared pointers.
         *
         * @param spath the save filepath
         */
        explicit WriteOut(std::filesystem::path spath)
        {
            save_path = spath;
            std::filesystem::create_directory(save_path);
        }

        /**
         * @brief The zDiff print helper function, writes each zDifference to an outFile
         * 
         * @param outFile the output stream to write to
         * @param zDiff the zDifference vector
         * @return true: if the write was successful
         * @return false: if the write was unsuccessful
         */
        bool printZDiff(
            std::ofstream &outFile,
            Eigen::VectorXf zDiff)
        {
            for (int i = 0; i < zDiff.rows(); i++)
            {
                outFile << zDiff(i) << std::endl;
            }
            return 0;
        }

        /**
         * @brief the comparisonMatrix print helper function, writes out "x: delta_x y: delta_y z: delta_z Times Seen: times_seen for each vertex"
         * 
         * @param outFile the output stream to write to
         * @param comparisonMatrix the matrix that holds the vector that a given vertex needs to move, and the amount of times it was "told" to move
         * @return true: if writeout was successful
         * @return false: if writeout wasn't successful
         */
        bool printComparisonMatrix(
            std::ofstream &outFile,
            Eigen::Matrix4Xf comparisonMatrix)
        {
            for (int i = 0; i < comparisonMatrix.cols(); i++)
            {
                outFile 
                    << "x: " 
                    << comparisonMatrix.col(i).x() 
                    << " y: " << comparisonMatrix.col(i).y() 
                    << " z: " << comparisonMatrix.col(i).z() 
                    << " Times Seen: " 
                    << comparisonMatrix.col(i).w() 
                    << std::endl; 
            }
            return 0;
        }

        /**
         * @brief Helper function to write point cloud and triangle data to file
         *
         * @param outFile the output stream to write to
         * @param sensed_points contains the x,y,z value of a given pointcloud
         * @param sensed_triangles contains corresponding triangles to sensed_point data
         * @return true: if writeout was successful
         * @return false: if writeout wasn't successful
         */
        static bool writePointCloudWithTriangleData(
            std::ofstream &outFile,
            forge_scan::PointMatrix sensed_points,
            forge_scan::TriangleVector sensed_triangles)
        {
            int success = 0;
            if (!outFile)
            {
                std::cerr << "Error: Could not open file for writing." << std::endl;
                success = -1;
            }
            else
            {
                for (int i = 0; i < sensed_points.cols(); i++)
                {
                    outFile 
                        << sensed_points(0, i) << " " 
                        << sensed_points(1, i) << " " 
                        << sensed_points(2, i) << " " 
                        << sensed_triangles.row(i) 
                        << std::endl;
                }
            }
            return success;
        }

        /**
         * @brief The PointCloud print helper function, prints out "x y z" for each point.
         *
         * @param outFile the output stream to write to
         * @param sensed_points contains the x,y,z value of a given pointcloud
         * @return true: if writeout was successful
         * @return false: if writeout wasn't successful
         */
        static bool printPointCloud(
            std::ofstream &outFile,
            forge_scan::PointMatrix sensed_points)
        {
            int success = 0;
            if (!outFile)
            {
                std::cerr << "Error: Could not open file for writing." << std::endl;
                success = -1;
            }
            else
            {
                for (int i = 0; i < sensed_points.cols(); i++)
                {
                    outFile << sensed_points(0, i) << " " << sensed_points(1, i) << " " << sensed_points(2, i) << std::endl;
                }
            }
            return success;
        }

        /**
         * @brief Writes out the current position of the camera as a translation and quaternion.
         *
         * @param camera_pose the current position of the camera as an Isometry3f
         * @param outFile the output stream to write to
         * @return true: if writeout was successful
         * @return false: if writeout wasn't successful
         */
        static bool writeCameraData(Eigen::Isometry3f camera_pose, std::ofstream &outFile)
        {
            Eigen::Quaternionf quaternion(camera_pose.rotation());
            outFile 
                << "Camera Translation: " 
                << camera_pose.translation().x() << " " 
                << camera_pose.translation().y() << " " 
                << camera_pose.translation().z() 
                << std::endl;
            outFile 
                << "Camera Rotation: " 
                << quaternion.x() << " " 
                << quaternion.y() << " " 
                << quaternion.z() << " " 
                << quaternion.w() 
                << std::endl;
            return 0;
        }

        std::filesystem::path save_path;
    };
} // end forge_scan namespace

#endif // FORGESCAN_FORWARDMODEL_WRITEOUT_HPP