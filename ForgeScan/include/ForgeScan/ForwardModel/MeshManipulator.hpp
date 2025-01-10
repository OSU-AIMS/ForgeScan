#ifndef FORGESCAN_FORWARDMODEL_MESHMANIPULATOR_HPP
#define FORGESCAN_FORWARDMODEL_MESHMANIPULATOR_HPP

#include "ForgeScan/Simulation/MeshLoader.hpp"
#include "ForgeScan/Simulation/GroundTruthScene.hpp"
#include "ForgeScan/Simulation/Scene.hpp"

#include <iostream>
#include <fstream>
#include <open3d/Open3D.h>
#include <open3d/core/Tensor.h>
#include <open3d/core/EigenConverter.h>

namespace forge_scan
{
    /**
     * @brief The MeshManipulator class, designed to handle all Mesh Manipulation and Mesh Calculations.
     *
     */
    class MeshManipulator
    {
    public:

        /**
         * @brief Creates a shared pointer to a MeshManipulator object.
         * 
         * @param scene The scene that the simulated mesh file is contained within
         * @return std::shared_ptr<MeshManipulator>: a shared pointer to a MeshManipulator object.
         */
        static std::shared_ptr<MeshManipulator> create(std::shared_ptr<forge_scan::simulation::Scene> scene)
        {
            return std::shared_ptr<MeshManipulator>(new MeshManipulator(scene));
        }

        /**
         * @brief Pulls the mesh out of the scene and stores it in a TriangleMesh
         *
         * @return open3d::t::geometry::TriangleMesh: the mesh file extracted from the scene
         */
        open3d::t::geometry::TriangleMesh extractMesh()
        {
            auto mesh_map = this->scene->getMeshMap();
            auto mesh_pair = mesh_map.begin();
            auto mesh = mesh_pair->second.second;

            return mesh;
        }

        /**
         * @brief Gets the vertex positions of a given mesh
         * 
         * @return Eigen::Matrix3Xf: A matrix with all the xyz positions of the vertices of the mesh
         */
        Eigen::Matrix3Xf getVertexPositions()
        {
            auto mesh = extractMesh();
            auto current_vertices = mesh.GetVertexPositions();
            auto vertex_positions_transpose = open3d::core::eigen_converter::TensorToEigenMatrixXf(current_vertices);
            Eigen::Matrix3Xf vertex_positions = Eigen::Transpose(vertex_positions_transpose);
            return vertex_positions;
        }

        /**
         * @brief Creates an .stl file of the current mesh in the scene
         * 
         * @param fpath the filepath to write the mesh to
         * @param iteration_count the iteration of the MBIR process 
         * @return std::filesystem::path: the location of the mesh
         */
        std::filesystem::path writeMesh(std::filesystem::path fpath, std::string mesh_name)
        {
            utilities::validateAndCreateFilepath(fpath, "", "", false);

            auto mesh = extractMesh();

            open3d::geometry::TriangleMesh legacy_mesh = mesh.ToLegacy();

            fpath += mesh_name;

            open3d::io::WriteTriangleMesh(fpath, legacy_mesh);

            return fpath;
        }

        /**
         * @brief Increases amount of triangles in a mesh file based on a given resolution
         *
         * UNFINISHED, ONLY FINISH IF TIME LEFT IN SPRINT
         *
         * @param scene
         * @return true
         * @return false
         */
        // static bool increaseResolution(std::shared_ptr<forge_scan::simulation::Scene> scene)
        // {
        //     // WORK IN PROGRESS, DON'T PUT MORE HOURS HERE
        //     int success = 0;

        //     auto mesh_map = scene->getMeshMap();
        //     // assuming only one mesh is in the scene
        //     auto mesh_pair = mesh_map.begin();
        //     auto mesh = mesh_pair->second.second;

        //     return success;
        // }

        /**
         * @brief Public comparison method, compares ground truth data to simulated data and outputs a "conversion" matrix
         * 
         * @param comparison_matrix the current comparison matrix to be added to
         * @param sensed_points the points sensed from the simulated data (x,y,z) tuple
         * @param sensed_triangles the corresponding triangles from each sensed_point
         * @param fpath the filepath to find the ground truth data
         * @param camera_pose the current pose of the camera as an Isometry3f
         * @param counter a counter to keep track of the image number
         * @return Eigen::Matrix4Xf a matrix that outlines how each vertex in the mesh needs to move to 
         * get closer to the desired object, as well as how many times it was told to move
         */
        Eigen::Matrix4Xf comparison(
            Eigen::Matrix4Xf comparison_matrix,
            forge_scan::PointMatrix sensed_points,
            forge_scan::TriangleVector sensed_triangles,
            std::filesystem::path fpath,
            Eigen::Isometry3f camera_pose,
            int counter)
        {
            Eigen::VectorXf zDiff = calculateZDiff(sensed_points, fpath, counter);
            return calculateComparisonMatrix(comparison_matrix, zDiff, sensed_triangles, camera_pose);
        }

        /**
         * @brief Gets the zDifference between ground truth and sensed_points
         * 
         * @param sensed_points the points sensed from the simulated data (x,y,z) tuple
         * @param fpath the filepath to find the ground truth data
         * @param counter a counter to keep track of the image number
         * @return Eigen::VectorXf: the zDifference vector
         */
        Eigen::VectorXf getZDiff(
            forge_scan::PointMatrix sensed_points,
            std::filesystem::path fpath,
            int counter)
        {
            return calculateZDiff(sensed_points, fpath, counter);
        }

        /**
         * @brief Divides the x,y,z positions in the matrix by how many times the vertex was seen, normalizing the matrix
         *
         * @param comparison_matrix the comparison matrix to be normalized, contains the total delta x,y,z to translate, 
         * and then the amount of times that vertex was told to move.
         * @param step_size the step size to multiply against the normalized matrix
         * @return Eigen::Matrix4Xf: the normalized comparison matrix
         */
        Eigen::Matrix4Xf normalizeComparisonMatrix(Eigen::Matrix4Xf comparison_matrix, float step_size)
        {
            Eigen::Matrix4Xf normalized_matrix(4, comparison_matrix.cols());
            normalized_matrix.setZero();
            for (int i = 0; i < comparison_matrix.cols(); i++)
            {
                if (comparison_matrix.col(i).w() != 0)
                {
                    normalized_matrix.col(i).x() = comparison_matrix.col(i).x() / comparison_matrix.col(i).w();
                    normalized_matrix.col(i).y() = comparison_matrix.col(i).y() / comparison_matrix.col(i).w();
                    normalized_matrix.col(i).z() = comparison_matrix.col(i).z() / comparison_matrix.col(i).w();
                    normalized_matrix.col(i).w() = comparison_matrix.col(i).w();
                }
            }
            normalized_matrix.row(0) * step_size;
            normalized_matrix.row(1) * step_size;
            normalized_matrix.row(2) * step_size;
            int while_iteration = 0;
            while(normalized_matrix.row(3).minCoeff()==0||while_iteration<100)
            {
                normalized_matrix = moveUnseenVertices(normalized_matrix, while_iteration);
                while_iteration++;
            }
            return normalized_matrix;
        }

        /**
         * @brief Updates the mesh in the scene based on the desired changed from the vertex_error matrix
         *
         * @param vertex_error the vector with x,y,z positions for each vertex translation
         */
        void meshUpdate(Eigen::Matrix3Xf vertex_error)
        {
            auto mesh = extractMesh();
            auto current_vertices = mesh.GetVertexPositions();
            auto eigen_current_vertices = open3d::core::eigen_converter::TensorToEigenMatrixXf(current_vertices);
            auto eigen_updated_matrix = eigen_current_vertices + Eigen::Transpose(vertex_error);
            auto updated_Matrix = open3d::core::eigen_converter::EigenMatrixToTensor(eigen_updated_matrix);
            mesh.SetVertexPositions(updated_Matrix);
            auto smoothed_mesh = smoothMesh(mesh,25);
            setMesh(smoothed_mesh);
        }

        /**
         * @brief gets the total percentage of vertexes that are above a certain accuracy threshold
         * 
         * @param comparison_matrix: the current comparison matrix to be analyzed
         * @param accuracy: the accuracy threshold to be compared against
         * @return float: the percentage of vertices that are above the accuracy threshold
         */
        float getTotalVeriticesOverThreshhold(Eigen::Matrix3Xf comparison_matrix, float accuracy)
        {
            int totalMovingVectors = 0;
            Eigen::VectorXf magnitudes(comparison_matrix.cols());
            for(int i = 0; i<comparison_matrix.cols(); i++)
            {
                magnitudes(i) = comparison_matrix.col(i).norm();
            }
            for(int i = 0; i<magnitudes.size(); i++)
            {
                if(magnitudes(i)<accuracy)
                {
                    totalMovingVectors++;
                }
            }
            std::cout << "Current Accuracy Percentage: " << float(totalMovingVectors)/magnitudes.size() << std::endl;
            return float(totalMovingVectors)/magnitudes.size();
        }

        /**
         * @brief Removes the "w" component of an Eigen::Matrix, 
         * in the case of this class this is the "Times Seen" component.
         * 
         * @param old_matrix the 4xN Matrix to be tersed to 3xN
         * @return Eigen::Matrix3Xf: the new 3xN matrix without the "Times Seen" component
         */
        Eigen::Matrix3Xf matrix4XfToMatrix3Xf(Eigen::Matrix4Xf old_matrix)
        {
            Eigen::Matrix3Xf new_matrix(3, old_matrix.cols());
            new_matrix << old_matrix.row(0), old_matrix.row(1), old_matrix.row(2);
            return new_matrix;
        }

    private:
        /**
         * @brief Private constructor to enforce use of shared pointers.
         *
         */
        explicit MeshManipulator(std::shared_ptr<forge_scan::simulation::Scene> scene)
        {
            this->scene = scene;
            createVertexTrianglesMatrix();
        }

        /**
         * @brief Creates a matrix that contains all the triangles that each vertex is a part of.
         * 
         */
        void createVertexTrianglesMatrix()
        {
            auto mesh = extractMesh();
            auto triangle_indices_tensor = mesh.GetTriangleIndices();
            auto triangle_indices = open3d::core::eigen_converter::TensorToEigenMatrixXf(triangle_indices_tensor);
            vertex_triangles.resize(triangle_indices.maxCoeff()+1);
            for(int i = 0; i < triangle_indices.rows(); i++)
            {
                vertex_triangles[triangle_indices(i,0)].push_back(i);
                vertex_triangles[triangle_indices(i,1)].push_back(i);
                vertex_triangles[triangle_indices(i,2)].push_back(i);
            }
        }

        /**
         * @brief Gets the neighbors of a given vertex
         * 
         * @param vertex the vertex to get the neighbors of
         * @return std::vector<int>: a vector of all the neighbors of the given vertex
         */
        std::vector<int> getVertexNeighbors(int vertex)
        {
            auto connected_triangles = vertex_triangles[vertex];
            auto mesh = extractMesh();
            auto triangle_indices_tensor = mesh.GetTriangleIndices();
            auto triangle_indices = open3d::core::eigen_converter::TensorToEigenMatrixXf(triangle_indices_tensor);
            std::vector<int> neighbors;
            for(int triangle:connected_triangles)
            {
                for(int i = 0; i < triangle_indices.cols(); i++)
                {
                    //contains equivalent
                    if(!(std::find(neighbors.begin(), neighbors.end(),triangle_indices(triangle,i)) 
                        != neighbors.end())&&triangle_indices(triangle,i)!=vertex)
                    {
                        neighbors.push_back(triangle_indices(triangle,i));
                    }
                }
            }
            return neighbors;
        }


        /**
         * @brief Calculates the z difference in the ground truth data and the simulated data
         *
         * @param sensed_points the simulated data
         * @param fpath the filepath to the ground truth data
         * @param counter the image number to be sampled from the ground truth data.
         * @return Eigen::VectorXf: a vector that contains the z difference value 
         * between the ground truth and the simulated data
         */
        Eigen::VectorXf calculateZDiff(
            forge_scan::PointMatrix sensed_points,
            std::filesystem::path fpath,
            int counter)
        {
            forge_scan::PointMatrix ground_points;
            Eigen::VectorXf zDiff;
            ground_points.setConstant(3, sensed_points.cols(), 0);
            zDiff.setConstant(sensed_points.cols(), 0);

            std::ifstream groundFile(std::string(fpath) + "groundtruth_pointcloud" + std::to_string(counter) + ".xyz");

            std::string line = "";
            int i = 0;
            while (std::getline(groundFile, line))
            {
                ground_points.col(i) << splitPointLine(line);
                i++;
            }

            //Setting zDiff to -11.0 to mark all points that were extrenous. Will be caught later
            for (int i = 0; i < sensed_points.cols(); i++)
            {
                if (ground_points.col(i).z() != 10)
                {
                    zDiff(i) = ground_points.col(i).z() - sensed_points.col(i).z();
                }
                else
                {
                    zDiff(i) = -11.0;
                }
            }
            return zDiff;
        }

        /**
         * @brief Based on the ZDifference, calculate the vector that each vertex needs to move along.
         *
         * @return Eigen::Matrix3Xf
         */

        /**
         * @brief Based on the ZDifference, calculates the un-normalized vector translation with total additions.
         * 
         * @param comparison_matrix the current comparison matrix to be added to
         * @param zDiff the zDifference vector
         * @param sensed_triangles the corresponding triangles from each zDiff
         * @param camera_pose the current pose of the camera as an Isometry3f
         * @return Eigen::Matrix4Xf: the updated comparison matrix
         */
        Eigen::Matrix4Xf calculateComparisonMatrix(
            Eigen::Matrix4Xf comparison_matrix,
            Eigen::VectorXf zDiff,
            forge_scan::TriangleVector sensed_triangles,
            Eigen::Isometry3f camera_pose)
        {
            auto mesh = extractMesh();
            auto tri_to_vertex_matrix = mesh.GetTriangleIndices();
            Eigen::Vector3f unit_vector = getUnitVector(camera_pose);

            for (int i = 0; i < zDiff.size(); i++)
            {
                if ((sensed_triangles(i) != (u_int32_t)-1)&&zDiff(i)!=-11)
                {
                    auto vertices = tri_to_vertex_matrix[sensed_triangles(i)].ToFlatVector<int64>();
                    for (int f : vertices)
                    {
                        comparison_matrix.col(f).x() = unit_vector.x() * zDiff(i) + comparison_matrix.col(f).x();
                        comparison_matrix.col(f).y() = unit_vector.y() * zDiff(i) + comparison_matrix.col(f).y();
                        comparison_matrix.col(f).z() = unit_vector.z() * zDiff(i) + comparison_matrix.col(f).z();
                        comparison_matrix.col(f).w() = comparison_matrix.col(f).w() + 1;
                    }
                }
            }
            return comparison_matrix;
        }

        /**
         * @brief Moves all unseen verticies in comparison matrix based on neighbors
         * 
         * @param comparison_matrix the current comparison matrix holding all "move" values
         * @param while_iterations the amount of times this algorithm has been applied to a single step
         * @return Eigen::Matrix4Xf: the new comparison matrix
         */
        Eigen::Matrix4Xf moveUnseenVertices(
            Eigen::Matrix4Xf comparison_matrix,
            int while_iterations
        )
        {
            Eigen::Matrix4Xf new_comparison_matrix(comparison_matrix.rows(), comparison_matrix.cols());
            new_comparison_matrix = comparison_matrix;
            for(int j = 0; j<comparison_matrix.cols(); j++)
            {
                if(comparison_matrix.col(j).w()==0)
                {
                    auto neighbors = getVertexNeighbors(j);
                    float vector_x = 0;
                    float vector_y = 0;
                    float vector_z = 0;
                    int count = 0;
                    for(int i:neighbors)
                    {
                        if(comparison_matrix.col(i).w()!=0)
                        {
                            vector_x += comparison_matrix.col(i).x();
                            vector_y += comparison_matrix.col(i).y();
                            vector_z += comparison_matrix.col(i).z();
                            count++;
                        }
                    }
                    if(count!=0)
                    {
                        new_comparison_matrix.col(j).x() = vector_x/count;
                        new_comparison_matrix.col(j).y() = vector_y/count;
                        new_comparison_matrix.col(j).z() = vector_z/count;
                        new_comparison_matrix.col(j).w() = -1 * while_iterations;
                    }
                }
            }
            return new_comparison_matrix;
        }

        /**
         * @brief Sets the given mesh to the new mesh in the scene
         *
         * @param mesh the new mesh to be stored
         */
        void setMesh(
            open3d::t::geometry::TriangleMesh mesh)
        {
            auto mesh_map = this->scene->getMeshMap();
            auto mesh_map_pair = mesh_map.begin();
            auto mesh_map_pair_key = mesh_map_pair->first;
            auto mesh_pair = mesh_map[mesh_map_pair_key];
            auto mesh_info = std::get<0>(mesh_pair);
            auto new_mesh_pair = std::make_pair(mesh_info, mesh);
            mesh_map[mesh_map_pair_key] = new_mesh_pair;
            this->scene->setMeshMap(mesh_map);
        }

        /**
         * @brief Smooths a given mesh based on the number of iterations
         * 
         * @param mesh the mesh to be smoothed
         * @param iterations: the number of iterations to smooth the mesh
         * @return open3d::t::geometry::TriangleMesh: the smoothed mesh 
         */
        open3d::t::geometry::TriangleMesh smoothMesh(open3d::t::geometry::TriangleMesh mesh, int iterations)
        {
            auto legacy_mesh = mesh.ToLegacy();
            legacy_mesh.FilterSmoothLaplacian(iterations, 1);
            auto new_mesh = open3d::t::geometry::TriangleMesh::FromLegacy(legacy_mesh);
            return new_mesh;
        }

        /**
         * @brief gets the unit vector from the current camera pose
         *
         * @param camera_pose the current pose of the camera as an Isometry3f
         * @return Eigen::Vector3f: a unit vector for the given camera pose
         */
        static Eigen::Vector3f getUnitVector(Eigen::Isometry3f camera_pose)
        {
            auto rotation_matrix = camera_pose.rotation();
            Eigen::Vector3f unit_vector(0, 0, 1);
            Eigen::Vector3f updated_unit_vector = rotation_matrix * unit_vector;
            return updated_unit_vector;
        }

        /**
         * @brief a helper function to read data from .xyz files, gets each line and outputs a point.
         *
         * @param line the current line of a text file as a string
         * @return forge_scan::Point: the current line converted to a point.
         */
        static forge_scan::Point splitPointLine(std::string line)
        {
            forge_scan::Point xyz;
            std::istringstream stream(line);
            stream >> xyz.x();
            stream >> xyz.y();
            stream >> xyz.z();
            return xyz;
        }

        std::shared_ptr<forge_scan::simulation::Scene> scene;
        std::vector<std::vector<int>> vertex_triangles;
    };
}

#endif
