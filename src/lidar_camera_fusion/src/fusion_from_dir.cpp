#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include "lidar_rgb_fusion.h"
#include "directory.h"
#include <iostream>

using namespace std;
using namespace cv;

Mat image_process(string image_path)
{
    Mat image = imread(image_path);
    cv_bridge::CvImage cv_Image;
    cv_Image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_Image.image = image;
    return image;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion_from_pcd");
    ros::NodeHandle nh;
    double min_distance, max_distance;
    std::string image_path, cloud_path, save_path;
    std::string intrinsic_path, extrinsic_path;
    
    // get parameters
    ros::param::get("intrinsic", intrinsic_path);
    ros::param::get("extrinsic", extrinsic_path);
    ros::param::get("min_distance", min_distance);
    ros::param::get("max_distance", max_distance);
    ros::param::get("image_path", image_path);
    ros::param::get("cloud_path", cloud_path);
    ros::param::get("save_path", save_path);

    // fusion method
    LidarRGBfusion fusion_node;
    fusion_node.set_intrinsic_path(intrinsic_path);
    fusion_node.set_extrinsic_path(extrinsic_path);
    fusion_node.set_min_distance(min_distance);
    fusion_node.set_max_distance(max_distance);
    fusion_node.get_parameters();

    // read cloud file list
    directory cloud_dir(cloud_path);
    cloud_dir.getFileName(cloud_path);
    vector<string> clouds = cloud_dir.getFileNameSet();
    ROS_INFO_STREAM(clouds.size());

    // read image file list
    directory image_dir(image_path);
    image_dir.getFileName(image_path);
    vector<string> images = image_dir.getFileNameSet();
    ROS_INFO_STREAM(clouds.size());
    
    // data process
    const char separator = '/';
    for (int i = 0; i < clouds.size();i++)
    {
        std::string val;
        std::vector<string> outputArray;
        std::stringstream streamData(clouds[i]);
        while (std::getline(streamData, val, separator))
        {
            outputArray.push_back(val);
        }
        // load cloud
        ROS_INFO_STREAM(clouds[i]);
        ROS_INFO_STREAM(images[i]);
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::io::loadPCDFile(clouds[i], cloud);
        // lodar image
        Mat image = image_process(images[i]);
        // fusion
        fusion_node.set_rgb_image(image);
        fusion_node.lidar_camera_registration(cloud);
        fusion_node.filter();
        pcl::PointCloud<pcl::PointXYZRGB> fused_cloud = fusion_node.return_point_cloud();
        //
        fused_cloud.height = fused_cloud.points.size();
        fused_cloud.width = 1;
        fused_cloud.is_dense = false;
        string file_save_path = save_path + '/' + outputArray.back();
        pcl::io::savePCDFile(file_save_path, fused_cloud);
        ros::Duration(1.0).sleep();
    }
    return 0;
}