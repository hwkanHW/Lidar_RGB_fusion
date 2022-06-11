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

using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion_from_pcd");
    ros::NodeHandle nh;
    std::string image_path, pcd_path, intrinsic_path, extrinsic_path, save_path;
    double min_distance, max_distance;
    //
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("RGB_cloud", 1);
    image_transport::ImageTransport it_(nh);
    image_transport::Publisher image_pub_ = it_.advertise("image", 1);
    //
    nh.getParam("image_path", image_path);
    nh.getParam("pcd_path", pcd_path);
    ros::param::get("intrinsic", intrinsic_path);
    ros::param::get("extrinsic", extrinsic_path);
    ros::param::get("min_distance", min_distance);
    ros::param::get("max_distance", max_distance);
    ros::param::get("save_path", save_path);
    //
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::io::loadPCDFile(pcd_path, cloud);
    //
    Mat image = imread(image_path);
    cv_bridge::CvImage cv_Image;
    cv_Image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_Image.image = image;
    image_pub_.publish(cv_Image.toImageMsg());
    //
    LidarRGBfusion fusion_node;
    fusion_node.set_intrinsic_path(intrinsic_path);
    fusion_node.set_extrinsic_path(extrinsic_path);
    fusion_node.set_min_distance(min_distance);
    fusion_node.set_max_distance(max_distance);
    fusion_node.get_parameters();
    //
    fusion_node.set_rgb_image(image);
    fusion_node.lidar_camera_registration(cloud);
    fusion_node.filter();
    pcl::PointCloud<pcl::PointXYZRGB> fused_cloud = fusion_node.return_point_cloud();
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(fused_cloud, msg);
    msg.header.frame_id = "map";
    cloud_pub.publish(msg);
    ros::Duration(1.0).sleep();
    //
    fused_cloud.height = fused_cloud.points.size();
    fused_cloud.width = 1;
    fused_cloud.is_dense = false;
    pcl::io::savePCDFile(save_path, fused_cloud);
    //
    ros::Duration(1.0).sleep();
    ros::shutdown();
}