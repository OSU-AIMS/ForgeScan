#include "scan_methods.hpp"

/**
 * Handles conversion from Image message to a depth image in meters
 * 
 * Filters out any values found as 0.01 and below in depth image to 10 meters away since those values
 * are seen as extraneous 
 * 
 * @param camera_image: a shared pointer for the camera_image message
 */
Eigen::MatrixXf ScanMethods::messageToEigen(const sensor_msgs::msg::Image::ConstSharedPtr& camera_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat image = cv_ptr->image;
    cv::Mat depth_image_meters;

    try 
    {
        cv_ptr = cv_bridge::toCvCopy(camera_image, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) 
    {
        std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        return Eigen::MatrixXf();
    }

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

void ScanMethods::runTurnTableReconstruction(const std::shared_ptr<forge_scan::sensor::Intrinsics> intr)
{
    std::shared_ptr<forge_scan::sensor::Camera> camera = forge_scan::sensor::Camera::create(intr, 0.0, 100);
    Eigen::Matrix3f K = Eigen::Matrix3f::Identity();
    forge_scan::PointMatrix sensed_points;
}