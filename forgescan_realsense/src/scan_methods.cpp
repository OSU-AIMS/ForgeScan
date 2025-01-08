#include "scan_methods.hpp"

/**
 * @brief Handles conversion from Image message to a depth image in meters
 * 
 * Filters out any values found as 0.01 and below in depth image to 10 meters away since those values
 * are seen as extraneous 
 * 
 * @param camera_image a shared pointer for the camera_image message
 * @return Eigen::MatrixXf 
 */
Eigen::MatrixXf ScanMethods::messageToEigen(const sensor_msgs::msg::Image::ConstSharedPtr& camera_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat depth_image_meters;

    try 
    {
        cv_ptr = cv_bridge::toCvCopy(camera_image, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) 
    {
        std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        return Eigen::MatrixXf();
    }
        
    cv::Mat image = cv_ptr->image;
    image.convertTo(depth_image_meters, CV_32F, 10.0);

    for (int i = 0; i < depth_image_meters.rows; ++i) 
    {
        for(int j = 0; j < depth_image_meters.cols; j++)
        {
            if(depth_image_meters.at<float>(i,j)<=1000)
            {
                depth_image_meters.at<float>(i,j) = 100000.0;
            }
            depth_image_meters.at<float>(i,j) = depth_image_meters.at<float>(i,j)/10000.0;
        }
    }

    Eigen::MatrixXf eigen_image(depth_image_meters.rows, depth_image_meters.cols);

    for(int i = 0; i < depth_image_meters.rows; ++i)
    {
        for(int j = 0; j < depth_image_meters.cols; ++j)
        {
            eigen_image(i, j) = depth_image_meters.at<float>(i,j);
        }
    }
    return eigen_image;
}

/**
 * @brief Runs a Turntable reconstruction with ForgeScan's turntable policy
 * 7 views, 3 tilts per reconstruction.
 * 
 * @param node the node to be used for logging
 * @param intr the intrinsics of the camera
 * @param manager the Forgescan manager that handles the Forgescan package
 * @param image_client the ros client that will call the camera_capture service
 */
void ScanMethods::runTurnTableReconstruction(
    const std::shared_ptr<rclcpp::Node> node,
    const std::shared_ptr<forge_scan::sensor::Intrinsics> intr, 
    const std::shared_ptr<forge_scan::Manager> manager, 
    const rclcpp::Client<forgescan_realsense::srv::CameraPose>::SharedPtr image_client
)
{
    std::shared_ptr<forge_scan::sensor::Camera> camera = forge_scan::sensor::Camera::create(intr, 0.0, 100);
    forge_scan::PointMatrix sensed_points;
    forge_scan::Extrinsic camera_pose;

    manager->policyAdd("--set-active --type Axis --n-views 7 --n-repeat 3 --x -1.0 --y -1.0 --z -1.0 --seed 50 --uniform");
    int picture_num = 0;
        while(!manager->policyIsComplete())
        {
            auto pose_request = std::make_shared<forgescan_realsense::srv::CameraPose::Request>();
            camera_pose = manager->policyGetView();

            Eigen::Matrix4f matrix_camera_pose = camera_pose.matrix();

            Eigen::Quaternionf quat(matrix_camera_pose.topLeftCorner<3,3>());
            
            auto camera_pose = geometry_msgs::msg::Pose();
            camera_pose.position.x = matrix_camera_pose(0,3);
            camera_pose.position.y = matrix_camera_pose(1,3);
            camera_pose.position.z = matrix_camera_pose(2,3);
            camera_pose.orientation.x = quat.x();
            camera_pose.orientation.y = quat.y();
            camera_pose.orientation.z = quat.z();
            camera_pose.orientation.w = quat.w();

            pose_request->pose = camera_pose;
            pose_request->picture_number = picture_num;

            //Add movement to move robot to position

            auto result_future = image_client->async_send_request(pose_request);
            
            if (rclcpp::spin_until_future_complete(node, result_future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto result = result_future.get();
                int len = result->eigenmatrix.size();
                sensed_points.resize(3, len);
                for(int i = 0; i<len; i++)
                {
                    sensed_points(0, i) = result->eigenmatrix[i].x;
                    sensed_points(1, i) = result->eigenmatrix[i].y;
                    sensed_points(2, i) = result->eigenmatrix[i].z;
                }
            } 
            else 
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to capture_image");
            }

            manager->reconstructionUpdate(sensed_points, camera->getExtr());

            manager->policyAcceptView();
            picture_num++;
        }
        RCLCPP_INFO(node->get_logger(), "Successfully finished Reconstruction");
}

geometry_msgs::msg::Pose ScanMethods::isometryToRosPose(Eigen::Isometry3f pose)
{
    Eigen::Matrix4f matrix_camera_pose = pose.matrix();

    Eigen::Quaternionf quat(matrix_camera_pose.topLeftCorner<3,3>());
    
    auto camera_pose = geometry_msgs::msg::Pose();
    camera_pose.position.x = matrix_camera_pose(0,3);
    camera_pose.position.y = matrix_camera_pose(1,3);
    camera_pose.position.z = matrix_camera_pose(2,3);
    camera_pose.orientation.x = quat.x();
    camera_pose.orientation.y = quat.y();
    camera_pose.orientation.z = quat.z();
    camera_pose.orientation.w = quat.w();

    return camera_pose;
}

/**
 * @brief Converts a geometry_msgs::msg::Pose into an Eigen::Isometry3f
 * 
 * @param pose the pose as a geometry_msgs::msg::Pose 
 * @return Eigen::Isometry3f: the same pose but as an Eigen::Isometry3f
 */
Eigen::Isometry3f ScanMethods::rosPoseToIsometry(geometry_msgs::msg::Pose pose)
{
    Eigen::Matrix4f transformation_matrix;
    Eigen::Quaternionf quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    transformation_matrix.setIdentity();
    transformation_matrix.block<3,3>(0,0) = quat.matrix();
    transformation_matrix.block<3,1>(0,3) = Eigen::Vector3f(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Isometry3f camera_pose = Eigen::Isometry3f(transformation_matrix);
    return camera_pose;
}


/**
 * @brief Tries to get camera intrinsics from intrinsics
 * 
 * @param node the node to log information from
 * @param intrinsics_client the ros client to call the intrinsics service
 * @return std::shared_ptr<forge_scan::sensor::Intrinsics>: the forge_scan intrinsics of the camera
 */
std::shared_ptr<forge_scan::sensor::Intrinsics> ScanMethods::try_to_get_camera_intrinsics(
    const std::shared_ptr<rclcpp::Node> node,
    const rclcpp::Client<forgescan_realsense::srv::Intrinsics>::SharedPtr intrinsics_client
    )
{
    std::shared_ptr<forge_scan::sensor::Intrinsics> intr;
    auto intrinsics_request = std::make_shared<forgescan_realsense::srv::Intrinsics::Request>();
    std::shared_ptr<forgescan_realsense::srv::Intrinsics::Response> intrinsics_result;
    auto result = intrinsics_client->async_send_request(intrinsics_request);
    if (result.valid() && rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) 
    {
        intrinsics_result = result.get();
        intr = forge_scan::sensor::Intrinsics::create(
            intrinsics_result->width, intrinsics_result->height, 
            intrinsics_result->mindepth, intrinsics_result->maxdepth, 
            intrinsics_result->fovx, intrinsics_result->fovy);
        RCLCPP_INFO(node->get_logger(), "Successfully retrieved intrinsics");
    } 
    else 
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to retrieve intrinsics values, using defaults");
        intr = forge_scan::sensor::Intrinsics::create();
    }
    return intr;
}