# FORGESCAN REALSENSE WRAPPER
<a href="https://www.open3d.org/docs/release/index.html"><img src="https://img.shields.io/badge/Open3D-0.18.0-gray?style=flat-square&logoColor=white&labelColor=black"/></a>
<a href="https://docs.ros.org/en/humble/index.html"><img src="https://img.shields.io/badge/ROS-Humble-blue?style=flat-square&logo=ROS&logoColor=white&labelColor=22314E"/></a>
<a href="https://releases.ubuntu.com/jammy/"><img src="https://img.shields.io/badge/Ubuntu-22.04-gray?style=flat-square&logo=ubuntu&logoColor=white&labelColor=E95420"/></a>

## Dependencies

### Open3D
This package depends on Open3D for processing and visualization. If not installed, follow the steps [here](https://www.open3d.org/docs/release/compilation.html#ubuntu-macos).

Note: If you **do not change the default install path**, you should run `sudo make install` when installing Open3D. By default, it will require `sudo` permissions to create the files.

#### If Open3D cmake has errors
If you encounter the following error when running `cmake ..` for Open3D:
```bash
CMake Error at CMakeLists.txt:1 (cmake_minimum_required):
  CMake 3.24 or higher is required
```

You will need to update your CMake version. However, if you `sudo apt remove` and `sudo apt install` to update it, this will break your ROS distro. 

To prevent breaks in the ROS distro, do the following:
- Download a new version of `cmake`, recommended to use the latest release [here](https://cmake.org/download/)
- Unzip the `cmake` download
- Build and install `cmake`
```bash
cd ~/Downloads/cmake-<maj>-<min>-<patch>/  # or wherever you downloaded cmake
./bootstrap --prefix=$HOME/cmake-install  # or wherever you'd like to install cmake
make
make install
export PATH=$HOME/cmake-install/bin:$PATH  # should match install path in line 2
export CMAKE_PREFIX_PATH=$HOME/cmake-install:$CMAKE_PREFIX_PATH  # should match install path in line 2
```
The last 2 lines can be placed in `.bashrc` if desired.

#### If Open3D cmake has no errors
In addition, add the installation path to your `CMAKE_PREFIX_PATH` environment variable to properly build the package.
```bash
export CMAKE_PREFIX_PATH=<path/to/open3d/install>/Open3D:$CMAKE_PREFIX_PATH  
# By default the path should be /usr/local/lib/cmake
```
You can add the install path to your `.bashrc` file if desired.

### ROS 2 Humble
This package depends on ROS 2 Humble or greater to build. 

Specifically, the `rosidl_get_typesupport_target` function is not introduced until Humble.

Pull down the dependent repos using `vcstools`:
```bash
vcs import < dependencies.repos
```

Install rosdep dependencies:
```bash
cd <forgescan_ws_dir>  # ROS workspace directory where this package is located
sudo rosdep init  # if first time using rosdep
rosdep update
rosdep install --from-paths src -y --ignore-src
```

Support for Foxy is possible following solution [here](https://github.com/TixiaoShan/LIO-SAM/issues/468). Foxy is not recommended because it is EOL. 
If you choose to use Foxy, include the `--include-eol-distros` argument when using `rosdep install`.

### Realsense ROS
Given that this is a realsense wrapper for an existing package, it is dependent on `realsense-ros`. Note that the version of `realsense-ros` is tied to the ROS distro. Therefore, if you are using a distro other than Humble, you should `git clone` the matching branch.

### Motoman Packages
The RViz visualization depends on motoman support packages, please configure the scene.urdf.xacro with a desired robot.

The current setup works with the [motorman_ms210_support] (https://github.com/EricMarcil/motoman/tree/GP225) package found in Eric Marcil's repo.

Please note that this requires [motoman_resources] (https://github.com/OSU-AIMS/aims-robots-all) which is found internally in aims-robots-all
