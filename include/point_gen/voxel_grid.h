# ifndef MY_VOXEL_H
# define MY_VOXEL_H

#include <point_gen/sim_sensor_reading.h>

#include <Eigen/Geometry>

#include <vector>
#include <string>


/// Convenience typedef for Eigen
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector3i Vector3i;

/// Convenience typedef for Eigen; 32 bit unsigned
typedef Eigen::Matrix<size_t, 3, 1> Vector3ui;


/// @brief Container for a 3 dimensional grid ov voxels.
class VoxelGrid
{
    private:
        /// Vector structure for the voxels. Increments fastest in X, then Y, then Z.
        std::vector<uint8_t> grid;

        /// Number of voxels that span the distance between each upper and lower bound
        Vector3ui space;

        /// Edge length of each voxel cube
        double resolution;

        /// Lower (X, Y, Z) bound for coordinates inside of grid
        Eigen::Vector3d lower;

        /// Upper (X, Y, Z) bound for coordinates inside of grid
        Vector3d upper;

        /// Pre-computed scaling factor for point placement
        Vector3d idx_scale;


        /// @brief Sets the voxel to the provided value
        /// @param idx The index of the voxel within the vector
        /// @param val The value to insert at the voxel index
        /// @return 0 if the voxel was accessed successfully or 1 if the index could not be accesses
        int set_voxel(const Vector3ui& idx, const uint8_t& val = 1);


        /// @brief Increases the value at the voxel location by 1
        /// @param idx The index of the voxel within the vector
        /// @return 0 if the voxel was incremented successfully; 1 if the index could not be accesses; or 2 
        ///         if the voxel was already at the maximum count.
        int inc_voxel(const Vector3ui& idx);


    public:
        /// If true will round all coordinates in space to the closest voxel, even if they
        /// would have been outside of the voxel space.
        bool round_points_in;


        /// @brief Constructs a voxel representation for the volume between the lower and upper coordinate bounds
        ///        with the given space in each direction.
        /// @param resolution The resolution of each voxel. Must be greater than 0.
        /// @param lower The point with the minimum X, Y, and Z coordinates for the voxelized space.
        /// @param upper The point with the maximum X, Y, and Z coordinates for the voxelized space.
        /// @param init Value to initialize all elements to
        /// @param round_points_in Controls how the grid deals with points outside the voxelized space. If true all points
        ///                        will be rounded into the closest voxel.
        /// @throws `invalid_argument` if the difference between upper and lower in any direction less than or equal to 0.
        /// @throws `invalid argument` if resolution is less than or equal to 0.
        ///         the respective lower bound.
        VoxelGrid(double resolution, Eigen::Vector3d lower, Eigen::Vector3d upper, const uint8_t& init = 0, bool round_points_in = true);


        /// @brief Transforms the input index into coordinates within the voxel grid.
        /// @param input Index location within the underlying std::vector
        /// @param output Index in discrete voxel space
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        /// @warning Transforming something from a vector index is hazardous. For space and grid
        ///          indicies that are out of bounds in X or Y directions, the calculated vector index
        ///          will be a valid alias for a point which is within the grid but with a different Z index. 
        /// @note Because of the above warning this function may be removed in the future.
        int gidx(const size_t& input, Vector3ui& output);


        /// @brief Transforms the input index into coordinates within the voxel grid.
        /// @param input Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @param output Index in discrete voxel space
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        int gidx(const Vector3d& input, Vector3ui& output);


        /// @brief Transforms the input index into coordinates within the continuous space that the grid spans
        /// @param input Index location within the underlying std::vector
        /// @param output Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        /// @warning Transforming something from a vector index is hazardous. For space and grid
        ///          indicies that are out of bounds in X or Y directions, the calculated vector index
        ///          will be a valid alias for a point which is within the grid but with a different Z index. 
        /// @note Because of the above warning this function may be removed in the future.
        int sidx(const size_t& input, Vector3d& output);


        /// @brief Transforms the input index into coordinates within the continuous space that the grid spans
        /// @param input Index in discrete voxel space
        /// @param output Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        int sidx(const Vector3ui& input, Vector3d& output);


        /// @brief Transforms the input index into the index location within the underlying std::vector
        /// @param input Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @param output Index location within the underlying std::vector
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        int vidx(const Vector3d& input, size_t& output);


        /// @brief Transforms the input index into the index location within the underlying std::vector
        /// @param input Index in discrete voxel space
        /// @param output Index location within the underlying std::vector
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        int vidx(const Vector3ui& input, size_t& output);


        /// @brief Checks that the given coordinates are within the grids coordinate bounds
        /// @param input Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @return True if all values are within their respective upper and lower bounds, false otherwise.
        bool const inline valid(const Vector3d input) {
            return (this->lower.array() <= input.array() && input.array() <= this->upper.array()).all();
        }


        /// @brief Checks that the given grid indicies are within the grids index bounds 
        /// @param input Index in discrete voxel space
        /// @return True if all values are within their respective upper and lower bounds, false otherwise.
        bool const inline valid(const Vector3ui input) {
            return (input.array() < this->space.array()).all();
        }


        /// @brief Checks that the given vector index is within the vector bounds
        /// @param input Index location within the underlying std::vector
        /// @return True if the value is less than the vector's size, false otherwise.
        bool const inline valid(const size_t& input) {
            return input < this->grid.size();
        }


        /// @brief Returns the value at the given location
        /// @param idx Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @return Value at the location
        uint8_t const inline at(const Vector3d& idx) {
            size_t vidx;
            this->vidx(idx, vidx);
            return this->at(vidx);       
        }


        /// @brief Returns the value at the given location
        /// @param idx Index in discrete voxel space
        /// @return Value at the location
        uint8_t const inline at(const Vector3ui& idx) {
            // TODO: This and the 
            size_t vidx;
            this->vidx(idx, vidx);
            return this->at(vidx);
        }


        /// @brief Returns the value at the given location
        /// @param idx Index location within the underlying std::vector
        /// @return Value at the index
        /// @throws `std::invalid_argument` if the vector index is out of bounds. 
        uint8_t const inline at(const size_t& idx) {
            if (!this->valid(idx)) throw std::invalid_argument("Input resulted in out of bound vector access.");
            return this->grid[idx];
        }


        /// @brief Sets the value at the given location
        /// @param idx Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @param val Value to set to
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the location is invalid.
        int inline set(const Vector3d& idx, const uint8_t& val) {
            size_t vidx;
            this->vidx(idx, vidx);
            return this->set(vidx, val);
        }


        /// @brief Sets the value at the given location
        /// @param idx Index in discrete voxel space
        /// @param val Value to set to
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the location is invalid.
        int inline set(const Vector3ui& idx, const uint8_t& val) {
            size_t vidx;
            this->vidx(idx, vidx);
            return this->set(vidx, val);
        }


        /// @brief Sets the value at the given location
        /// @param idx Index location within the underlying std::vector
        /// @param val Value to set to
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        int inline set(const size_t& idx, const uint8_t& val) {
            if ( !this->valid(idx) ) return -1;
            this->grid[idx] = val;
            return 0;
        }


        /// @brief Increments the value at the given location
        /// @param idx Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @param val Value to Increments by. May be negative.
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the location is invalid.
        /// @warning This does not check for overflow or underflow.
        int inline inc(const Vector3d& idx, const uint8_t& val = 1) {
            size_t vidx;
            this->vidx(idx, vidx);
            return this->inc(vidx, val);       
        }


        /// @brief Increments the value at the given location
        /// @param idx Index in discrete voxel space
        /// @param val Value to Increments by. May be negative.
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the location is invalid.
        /// @warning This does not check for overflow or underflow.
        int inline inc(const Vector3ui& idx, const int& val = 1) {
            size_t vidx;
            this->vidx(idx, vidx);
            return this->inc(vidx, val);       
        }


        /// @brief Increments the value at the given location
        /// @param idx Index location within the underlying std::vector
        /// @param val Value to Increments by. May be negative.
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the location is invalid.
        /// @warning This does not check for overflow or underflow.
        int inline inc(const size_t& idx, const int& val = 1) {
            if ( !this->valid(idx) ) return -1;
            this->grid[idx] += val;
            return 0;
        }


        /// @brief Sets the given point in space to the provided value
        /// @param point The X, Y, Z coordinate in space.
        /// @param val The value to set the voxel that point is in to. Default is 1.
        /// @return 0 if the point's value was added successfully; or 1 if the point fell outside the voxel grid.
        int add_point(const Eigen::Vector3d& point, const uint8_t& val = 1);

        
        /// @brief Increases the value at the point's location by 1
        /// @param point The X, Y, Z coordinate in space.
        /// @return 0 if the point's value was added successfully; or 1 if the point fell outside the voxel grid.
        int inc_point(const Eigen::Vector3d& point);


        /// @brief Adds equally-spaced points along the line, including the start and end
        /// @param start Coordinate to start from
        /// @param end Coordinate to end at
        /// @param num The number of points to add. Minimum of two
        /// @param surface Value to set elements on the surface (end of the line) to
        /// @param line Value to set elements along the line to.
        /// @return 0 if the line was added without issue
        int add_linear_space(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const size_t& num = 2, const uint8_t &surface = 1, const uint8_t &line = 2);


        /// @brief Adds a line between the start and end point that places approximately one point in each voxel the line touches
        /// @param start Coordinate to start from
        /// @param end Coordinate to end at
        /// @param vox_res TODO: unsure how to treat this tuning variable yet
        /// @param surface Value to set elements on the surface (end of the line) to
        /// @param line Value to set elements along the line to.
        /// @return 0 if the line was added without issue
        /// @note TODO: This is NOT IMPLEMENTED YET
        int add_line_fast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const double& vox_res, const uint8_t& surface = 1, const uint8_t& line = 2);


        /// @brief Adds the provided sensor to the grid
        /// @param scanner SimSensorReading object to add
        /// @return 0 if the scanner was added without issue
        int add_sensor(const SimSensorReading& scanner, const uint8_t& surface = 1, const uint8_t& line = 2);


        /// @brief Saves the grids points in a CSV-line format.
        ///        The values are stored incrementing in fastest in  X, then Y, then Z.
        /// @param fname The name for the save file. 
        void save_csv(const std::string& fname);


        /// @brief Saves in the HDF5 Format
        /// @param fname File name
        void save_hdf5(const std::string& fname);


        /// @brief Loads data from an HDF5 file
        /// @param fname File name
        void load_hdf5(const std::string& fname);


        /// @brief Transforms a grid coordinate to the index within the actual vector.
        /// @param grid_idx The X, Y, Z coordinate in the voxel grid.
        /// @return The 1D vector index for the provided 3D grid index.
        /// @note This DOES NOT check validity of that index position within the grid.
        size_t inline grid_idx_to_vector_idx(const Vector3ui& grid_idx){
            return ( grid_idx[0] ) + ( grid_idx[1] * this->space[0] ) + ( grid_idx[2] * this->space[0] * this->space[1] );
        };


        /// @brief Turns a vector list index into a a grid coordinate
        /// @param vector_idx Index in the vector
        /// @returns X, Y, Z coordinates within the voxel grid
        Vector3ui vector_idx_to_grid_idx(const size_t& vector_idx);


        /// @brief Turns coordinates in space into coordinates inside the voxel grid
        /// @param space_xyz The X, Y, Z coordinates in space
        /// @param grid_idx The X, Y, Z returned coordinates in the grid
        /// @return 0 if the grid index is valid, or 1 if it is invalid.
        /// @warning Does not check if the index was out of bounds. See `voxel_in_grid`
        int space_to_grid_idx(const Eigen::Vector3d& space_xyz, Vector3ui& grid_idx);
        

        /// @brief Queries the voxel grid to return the value stored at in the specified space
        /// @param space_xyz The coordinate in space
        /// @return Value at the voxel grid for the point in space. Or 0 if the space was not inside the voxel grid.
        uint8_t space_at(const Eigen::Vector3d& space_xyz);


        /// @brief Queries the space falls within the voxel grid
        /// @param space_xyz The coordinate in space
        /// @return True if the space falls withing the bounds of the voxel grid or False otherwise.
        bool space_in_grid(const Eigen::Vector3d& space_xyz);


        /// @brief Queries the voxel grid to return the value stored at in the specified index
        /// @param voxel_idx The XYZ indices in the voxel grid.
        /// @return Value at the voxel grid for the index. Or 0 if the index was not inside the voxel grid.
        /// @warning Does not check if the index was out of bounds. See `voxel_in_grid`
        uint8_t voxel_at(const Vector3ui& voxel_idx);


        /// @brief Queries the index falls within the voxel grid
        /// @param voxel_idx The XYZ indices in the voxel grid.
        /// @return True if the space falls withing the bounds of the voxel grid or False otherwise.
        bool voxel_in_grid(const Vector3ui& voxel_idx);

        /// @brief Returns the voxel list indicies of the 6 connected neighbors
        /// @param grid_idx The input grid space to find the connections of.
        /// @param connected Target output for the connect voxel-list neighbors.
        /// @note This does NOT verify if the indicies are valid on the voxel list.
        /// @note This does NOT turn these into X, Y, Z coordinates in the Voxel Grid or World space.
        ///       If this is the desired format then you must convert them.
        void get_6_connect_vector_list_idx(const Vector3ui& grid_idx, std::vector<size_t>& connected);
};


# endif // MY_VOXEL_H
