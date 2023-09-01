#ifndef FORGE_SCAN_RECONSTRUCTION_RAY_TRACING_GRID_TRAVERSAL_HPP
#define FORGE_SCAN_RECONSTRUCTION_RAY_TRACING_GRID_TRAVERSAL_HPP

#include <cstddef>

#include "ForgeScan/Common/AABB.hpp"
#include "ForgeScan/Common/Grid.hpp"
#include "ForgeScan/Common/VectorMath.hpp"


namespace forge_scan {
namespace data {
namespace ray_trace {


// *********************************************************************************************************************** //
// * This implements the ray traversal of a Voxel Grid, as used by the Reconstruction class.                             * //
// *                                                                                                                     * //
// * The methods in this file follow the the Amanatides-Woo algorithm for fast voxel traversal. See:                     * //
// *     http://www.cse.yorku.ca/~amana/research/grid.pdf                                                                * //
// *                                                                                                                     * //
// * Essentially, this treats the input ray as a parametrized line between the start and end point. The algorithm must   * //
// * decide what direction - X, Y, or Z - to step in. For each direction, the track a parameter, `dist`, for the         * //
// * distance traveled from the starting voxel in that direction. The direction that has the smallest `dist` value is    * //
// * selected to `step` in. This either increments or decrements the index in that direction.                            * //
// *                                                                                                                     * //
// * When a step is take the `dist` value for that direction is updated. This update, `delta`, is equal to the           * //
// * voxel resolution in that direction divided by the normal of the ray. This describes how many units of the vector    * //
// * must be walked to traverse one voxel unit.                                                                          * //
// *                                                                                                                     * //
// * For example if the `normal` in a direction is large (meaning the vector nearly points along the direction), the     * //
// * `delta` will be small and the algorithm will take many steps in that direction before choosing another one.         * //
// *                                                                                                                     * //
// * It is critical for this method that the grid of voxels may is considered as an axis-aligned bounding-box (AABB).    * //
// * This means any input data must be properly transformed before reaching this algorithm.                              * //
// *                                                                                                                     * //
// * The algorithm also considers that the voxel "origin" is the lower-bounding corner of the voxel. If the rest of the  * //
// * code considers the "origin" to be the center of the voxel then a reference frame shift of half the voxel resolution * //
// * must be applied to the ray's start point when calculating `dist`. See `correct_traversal_parameters` for more.      * //
// *********************************************************************************************************************** //


/// @brief Finds the first item in the trace with a distance greater than the specified value.
/// @param ray_trace A shared, constant trace to search through.
/// @param min_dist  Lower bound distance to find.
/// @return Constant iterator for `ray_trace` which points either to the first element greater than
///         `min_dist` or to the end of ray_trace.
inline trace::const_iterator first_above_min_dist(std::shared_ptr<const trace> ray_trace, const float& min_dist)
{
    trace::const_iterator iter = ray_trace->begin();
    while (iter != ray_trace->end())
    {
        if (iter->second >= min_dist)
        {
            return iter;
        }
        ++iter;
    }
    return iter;
}


/// @brief Helper for `get_ray_trace`.
/// @warning This should only be called by `get_ray_trace`.
inline int get_step(const std::ptrdiff_t& d, const std::ptrdiff_t* sign)
{
    static constexpr int STEP_DIR[2] = {1, -1};
    return STEP_DIR[sign[d]];
}


/// @brief Helper for `get_ray_trace`.
/// @warning This should only be called by `get_ray_trace`.
inline float get_delta(const std::ptrdiff_t& d, const Direction& inv_normal,
                       const std::shared_ptr<const Grid::Properties>& properties)
{
    return std::abs(properties->resolution * inv_normal[d]);
}


/// @brief Helper for `get_ray_trace`.
/// @warning This should only be called by `get_ray_trace`.
inline float get_dist(const std::ptrdiff_t& d, const std::ptrdiff_t* sign, const Index& c_idx,
                      const Point& sensed_adj, const Direction& inv_normal, const float& dist_min_adj,
                      const std::shared_ptr<const Grid::Properties>& properties)
{
    static constexpr float NEXT_ADJ[2] = {0.5, -0.5};
    return dist_min_adj + ((c_idx[d] + NEXT_ADJ[sign[d]]) * properties->resolution - sensed_adj[d]) * inv_normal[d];
}


/// @brief Helper for `get_ray_trace`.
/// @warning This should only be called by `get_ray_trace`.
inline std::ptrdiff_t get_min_dist(const float* dist)
{
    static constexpr std::ptrdiff_t X = 0, Y = 1, Z = 2;
    return (dist[X] < dist[Y] && dist[X] < dist[Z]) ? X : (dist[Y] < dist[Z]) ? Y : Z;
}


/// @brief Calculates what voxels are hit on the ray between `sensed` and `origin`.
/// @param ray_trace[out] A trace of what voxels were hit and the distance from that voxel to the `sensed` voxel. 
/// @param sensed Sensed point, the start of the ray.
/// @param origin Origin point, the end of the ray.
/// @param properties Shared Grid Properties for the Voxel Grids begin traversed.
/// @param dist_min Minimum distance to trace along the ray, relative to the `sensed` point.
/// @param dist_min Maximum distance to trace along the ray, relative to the `sensed` point.
/// @return True if the ray intersected the Grid, this indicates that `ray_trace` has valid data to add.
inline bool get_ray_trace(std::shared_ptr<trace>& ray_trace,
                          const Point& sensed, const Point& origin,
                          const std::shared_ptr<const Grid::Properties>& properties,
                          const float& dist_min, const float& dist_max)
{
    static constexpr std::ptrdiff_t X = 0, Y = 1, Z = 2;

    ray_trace->clear();

    // Adjusted min and max distances so we only trace the ray while it is within the Grid's bounds.
    float dist_min_adj, dist_max_adj;

    float length;
    Direction normal, inv_normal;
    vector_math::get_length_normal_and_inverse_normal(sensed, origin, length, normal, inv_normal);
    length = std::min(length, dist_max);
    
    const bool valid_intersection = AABB::fast_eigen_find_bounded_intersection(properties->dimensions, sensed, inv_normal,
                                                                               dist_min, length, dist_min_adj, dist_max_adj);
    if (valid_intersection)
    {
        dist_min_adj = std::max(dist_min_adj, dist_min);
        dist_max_adj = std::min(dist_max_adj, dist_max);

        const Point sensed_adj = sensed + normal * dist_min_adj;
        Index c_idx = properties->pointToIndex(sensed_adj);

        const std::ptrdiff_t sign[3] = {std::signbit(normal[X]), std::signbit(normal[Y]), std::signbit(normal[Z])};

        // Direction of travel (increment or decrement) along the respective axis.
        const int step[3] = { get_step(X, sign), get_step(Y, sign), get_step(Z, sign) };

        // The amount of distance to move one voxel length along each axis based on the ray's direction.
        const float delta[3] = { get_delta(X, inv_normal, properties),
                                 get_delta(Y, inv_normal, properties),
                                 get_delta(Z, inv_normal, properties) };

        // Cumulative distance traveled along the respective axis.
        float dist[3] = { get_dist(X, sign, c_idx, sensed_adj, inv_normal, dist_min_adj, properties),
                          get_dist(Y, sign, c_idx, sensed_adj, inv_normal, dist_min_adj, properties),
                          get_dist(Z, sign, c_idx, sensed_adj, inv_normal, dist_min_adj, properties) };

        try
        {
            ray_trace->emplace_back(properties->at(c_idx), dist_min_adj);

            std::ptrdiff_t i = get_min_dist(dist);
            while (dist[i] <= dist_max_adj)
            {
                c_idx[i] +=  step[i];
                ray_trace->emplace_back(properties->at(c_idx), dist[i]);

                dist[i]  += delta[i];
                i = get_min_dist(dist);
            }
        }
        catch (const std::out_of_range& e)
        {
            // Algorithm should never go out of bounds. But catching here dose not impact performance and prevents
            // undefined behavior and silent errors in a Voxel Grid update where all indicies are assumed to be valid.
            const std::string e_what(e.what());
            throw std::out_of_range("Ray tracing failed. This should not happen. Failed with: " + e_what);
        }
    }
    return valid_intersection;
}


} // namespace ray_trace
} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTION_RAY_TRACING_GRID_TRAVERSAL_HPP
