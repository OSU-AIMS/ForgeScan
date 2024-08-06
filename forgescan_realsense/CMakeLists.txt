cmake_minimum_required(VERSION 3.8)
project(forgescan_realsense)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Define FORGE_SCAN_MESHES_DIR
add_definitions(-DFORGE_SCAN_MESHES_DIR="/home/bturner86239/forgescan_ws/ForgeScan/share/Meshes")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(HDF5 REQUIRED)
find_package(Open3D REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(cv_bridge)

set(msg_files
    "msg/Intrinsics.msg"
    "msg/EigenVector.msg"
    "srv/Intrinsics.srv"
    "srv/ToTransform.srv"
    "srv/CameraPose.srv"
    DEPENDENCIES geometry_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
)

# Define the executables and their dependencies

# realsense_handler executable
add_executable(realsense_handler src/realsense_handler.cpp)
ament_target_dependencies(
    realsense_handler 
    rclcpp 
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
)

# scan_manager executable
add_executable(scan_manager src/scan_manager.cpp)
ament_target_dependencies(
    scan_manager
    rclcpp 
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    Open3D
    std_srvs
)

# scan_image executable
add_executable(scan_image src/scan_image.cpp src/scan_methods.cpp)
ament_target_dependencies(
    scan_image
    rclcpp
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2
    Open3D
    std_srvs
    cv_bridge
)

# Include directories for dependencies
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${HDF5_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/hdf5/serial
    /usr/include/eigen3
    /home/bturner86239/forgescan_ws/src/ForgeScan/include
    /home/bturner86239/forgescan_ws/src/ForgeScan/deps/HighFive/include
)

# Link directories for dependencies
link_directories(
    ${HDF5_LIBRARY_DIRS}
    /usr/lib/x86_64-linux-gnu/hdf5/serial
)

# Install the executables
install(TARGETS
    realsense_handler
    scan_manager
    scan_image
    DESTINATION lib/${PROJECT_NAME})

# Generate typesupport
rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)

# Link libraries for the executables
target_link_libraries(realsense_handler 
    "${cpp_typesupport_target}"
)

target_link_libraries(scan_manager
    "${cpp_typesupport_target}"
    "${HDF5_LIBRARIES}"
    "${OpenCV_LIBS}"
)

target_link_libraries(scan_image
    "${cpp_typesupport_target}"
    "${HDF5_LIBRARIES}"
    "${OpenCV_LIBS}"
    "${cv_bridge_LIBRARIES}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
