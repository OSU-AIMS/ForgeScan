#ifndef FORGE_SCAN_VOXELSTEP_HPP
#define FORGE_SCAN_VOXELSTEP_HPP

#include "ForgeScan/Common/RayTrace.hpp"
#include "ForgeScan/Common/VoxelData.hpp"
#include "ForgeScan/Data/VoxelGrids/Binary.hpp"
#include "ForgeScan/Data/Reconstruction.hpp"

namespace forge_scan
{

    namespace data
    {
        // Forward definition to allow friend access.
        class Reconstruction;

        class VoxelStep
        {

            public:

            static std::shared_ptr<VoxelStep> create()
            {
                return std::shared_ptr<VoxelStep>(new VoxelStep());
            }

            /**
             * Steps over voxels in a ray given a origin and sensed point, then gathers all Voxel Indicies until an occupied voxel.
             * 
             * @param reconstruction: a shared pointer to the reconstruction object
             * @param sensed: the point in the voxelgrid that is wanting the ray to be cast to
             * @param origin: the point in the voxelgrid that the ray is originating from
             * @return a vector<size_t> filled with the indexes of all voxels that are traced over by a ray until a voxel that is occupied is reached
             */
            std::vector<size_t> greedyStepVoxels(const std::shared_ptr<const forge_scan::data::Reconstruction> reconstruction, const Point& sensed, const Point& origin)
            {
                std::vector<size_t> greedy_voxels;

                auto voxel_grid_ = reconstruction->getChannelView("binary");
                auto voxel_data = voxel_grid_->getData();
                auto typed_voxel_data = std::get<std::vector<uint8_t>>(voxel_data);
                if (get_ray_trace(ray_trace, sensed, origin, reconstruction->grid_properties,
                                  reconstruction->getMinDist(), reconstruction->getMaxDist()))  
                    {
                    if (!ray_trace->empty())
                    {
                        size_t j = 0;
                        std::cout << "Ray Trace Size: " <<ray_trace->size() << std::endl;
                        while (j < ray_trace->size() &&
                            typed_voxel_data.at(ray_trace->at(j).i) != VoxelOccupancy::TYPE_OCCUPIED)
                        {
                            std::cout << "Ray Trace at: " <<ray_trace->at(j).i << std::endl;
                            greedy_voxels.push_back(ray_trace->at(j).i);
                            j++;
                        }
                    }
                }

                return greedy_voxels;
            }

            /**
             * Steps over voxels in a ray given a origin and sensed point, then gathers all Voxel Indicies until an occupied or unknown voxel.
             * 
             * @param reconstruction: a shared pointer to the reconstruction object
             * @param sensed: the point in the voxelgrid that is wanting the ray to be cast to
             * @param origin: the point in the voxelgrid that the ray is originating from
             * @return a vector<size_t> filled with the indexes of all voxels that are traced over by a ray until a voxel that is occupied or unknown is reached
             */
            std::vector<size_t> conservativeStepVoxels(const std::shared_ptr<const forge_scan::data::Reconstruction> reconstruction, const Point& sensed, const Point& origin)
            {
                std::vector<size_t> conservative_voxels;
                auto voxel_grid_ = reconstruction->getChannelView("binary");  // Ensure this argument is correct
                forge_scan::VectorVariant voxel_data = voxel_grid_->getData();

                if (get_ray_trace(ray_trace, sensed, origin, reconstruction->grid_properties,
                                  reconstruction->getMinDist(), reconstruction->getMaxDist()))
                { //LEARN HOW TO USE GDB, THIS WILL HELP YOU SO MUCH
                    if (!ray_trace->empty())
                    {
                        size_t j = 0;
                        std::cout << "The trace isn't empty!" << std::endl;
                        while (j < ray_trace->size() &&
                                std::get<std::vector<long unsigned int>>(voxel_data).at(ray_trace->at(j).i) != VoxelOccupancy::TYPE_OCCUPIED &&
                                std::get<std::vector<long unsigned int>>(voxel_data).at(ray_trace->at(j).i) != VoxelOccupancy::TYPE_UNKNOWN)
                        {
                            std::cout << ray_trace->at(j).i << std::endl;
                            conservative_voxels.push_back(ray_trace->at(j).i);
                            j++;
                        }
                    }
                }

                return conservative_voxels;
            }

            std::shared_ptr<Trace> ray_trace;

            private:

            explicit VoxelStep(): ray_trace(std::make_shared<Trace>())
            {

            }
        };

    }

}

#endif //FORGE_SCAN_VOXELSTEP_HPP
