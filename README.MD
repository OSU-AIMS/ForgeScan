# FORGESCAN REALSENSE WRAPPER
<a href="https://www.open3d.org/docs/release/index.html"><img src="https://img.shields.io/badge/Open3D-0.18.0-gray?style=flat-square&logoColor=white&labelColor=black"/></a>
<a href="https://docs.ros.org/en/humble/index.html"><img src="https://img.shields.io/badge/ROS-Humble-blue?style=flat-square&logo=ROS&logoColor=white&labelColor=22314E"/></a>
<a href="https://releases.ubuntu.com/jammy/"><img src="https://img.shields.io/badge/Ubuntu-22.04-gray?style=flat-square&logo=ubuntu&logoColor=white&labelColor=E95420"/></a>

## Hardware

Based on an Intel Realsense D455 is the realsense camera used for this project. It is required to be plugged in to run the main reconstruction service.

## FORGESCAN DOCUMENTATION

[ForgeScan Github.io](https://schellenberg3.github.io/ForgeScan/)

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

#### Error mentioning "usr/lib/llvm-11/lib/libc++.so" 

 If the above error occurs, rebuild OPEN3D from source.

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

The current setup works with the [motorman_ms210_support](https://github.com/EricMarcil/motoman/tree/GP225) package found in Eric Marcil's repo.

Please note that this requires [motoman_resources](https://github.com/OSU-AIMS/aims-robots-all) which is found internally in aims-robots-all

Pull both of these files down into the main directory alongside the other ROS packages in order to run.

Both can be cloned from [aims-robots-all](https://github.com/OSU-AIMS/aims-robots-all)

### OpenCV2

OpenCV2 needs to be built from source according to the following [instructions](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)

## Further Development and Use with Robot

For development and use with a robot, within the 'scan_methods.cpp' file, add the controls to move the robot to the 'camera_pose' position [here](https://github.com/OSU-AIMS/ForgeScan/blob/dev/ForgeScan_Implementation/forgescan_realsense/src/scan_methods.cpp#L91-L98)

```
  camera_pose.orientation.w = quat.w();

  pose_request->pose = camera_pose;
  pose_request->picture_number = picture_num;

  //Add movement to move robot to position

  auto result_future = image_client->async_send_request(pose_request);
```

## Examples

For basic example of functions, please see the ForgeScan/src/Examples to use ForgeScan
  
  The scene file will set up the scene of ForgeScan, putting a part into a virtual environment in order to be scaned by a virtual camera
  The Reconstruction file will run a full reconstruction with 10 images.
  Both can be run after building ForgeScan, but only the hpp files (not the cpp/executables) are used for the ROS wrapper

The ROS Wrapper can be ran with the command
```
ros2 launch forgescan_realsense forgescan_realsense.launch.xml
```

This will launch all nodes required to run the reconstruction service, alongside a visualization tool for an MS210, the robot will not move in RVIZ as this software has yet to be tested on hardware, but will in the future. 

The main ros service to call is '/forgescan_realsense/reconstruction' within cpp or python code. The service requires an empty service message to run.
To call directly from the command line, please use
```
ros2 service call /forgescan_realsense/reconstruction std_srvs/srv/Empty
```


