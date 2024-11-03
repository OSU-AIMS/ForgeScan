# FORGESCAN REALSENSE WRAPPER
<img src="https://img.shields.io/badge/ROS 2-Humble-blue"/></a> <a href="[https://docs.ros.org/en/Ubuntu 22.04/index.html](https://releases.ubuntu.com/jammy/)"><img src="https://img.shields.io/badge/Ubuntu-22.04-orange"/></a> <a href="[https://docs.ros.org/en/Ubuntu 22.04/index.html](https://releases.ubuntu.com/jammy/)"><img src="https://img.shields.io/badge/Open3D-0.18.0-purple"/></a> 

## Dependencies

### ROS 2 Humble
This package depends on ROS 2 Humble or greater to build. 

Specifically, the `rosidl_get_typesupport_target` function is not introduced until Humble.

Support for Foxy is possible following solution [here](https://github.com/TixiaoShan/LIO-SAM/issues/468). Foxy is not recommended because it is EOL. 
If you choose to use Foxy, include the `--include-eol-distros` argument when using `rosdep install`.

### Realsense ROS
Given that this is a realsense wrapper for an existing package, it is dependent on `realsense-ros`. Note that the version of `realsense-ros` is tied to the ROS distro. Therefore, if you are using a distro other than Humble, you should `git clone` the matching branch.

### Open3D
This package depends on Open3D for processing and visualization. If not installed, follow the steps [here](https://www.open3d.org/docs/release/compilation.html#ubuntu-macos).

In addition, add the installation path to your `CMAKE_PREFIX_PATH` environment variable to properly build the package.
```bash
export CMAKE_PREFIX_PATH=<path/to/open3d/install>/Open3D:$CMAKE_PREFIX_PATH  
# By default the path should be /usr/local/lib/cmake
```
You can add the install path to your `.bashrc` file if desired.

### Motoman Packages
The RViz visualization depends on motoman support packages, please configure the scene.urdf.xacro with a desired robot.

The current setup works with the [motorman_ms210_support] (https://github.com/EricMarcil/motoman/tree/GP225) package found in Eric Marcil's repo.

Please note that this requires [motoman_resources] (https://github.com/OSU-AIMS/aims-robots-all) which is found internally in aims-robots-all