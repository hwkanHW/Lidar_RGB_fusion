#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace cv;

Mat color_img;
pcl::PointCloud<pcl::PointXYZI> points, fused_cloud;
std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud;

void image_callback(const sensor_msgs::Image &msg){
    cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    color_img = img->image;
}

void lidar_callback(const sensor_msgs::PointCloud2::Ptr &msg){
    pcl::fromROSMsg(*msg, points);
    cloud.push_back(points);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "lidar_camera_collection");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    int size_lidar;
    std::string image_topic, lidar_topic;
    nh.getParam("image_topic", image_topic);
    nh.getParam("lidar_topic", lidar_topic);
    nh.getParam("size_lidar", size_lidar);

    ros::Subscriber image_sub = nh.subscribe(image_topic, 1, image_callback);
    ros::Subscriber lidar_sub = nh.subscribe(lidar_topic, 1, lidar_callback);
    
    while (ros::ok())
    {
        if(cloud.size() >= size_lidar){
            ROS_INFO_STREAM("waiting for lidar data.");
            break;
        }
    }
    ROS_INFO_STREAM(cloud.size());

    for (int i = 0; i < cloud.size(); i++)
    {
        for (int j = 0; j < cloud[i].size();j++){
            fused_cloud.points.push_back(cloud[i].points[j]);
        }
    }
    fused_cloud.height = fused_cloud.points.size();
    fused_cloud.width = 1;
    pcl::io::savePCDFile("/home/hwkan/lidar.pcd", fused_cloud);

    imwrite("/home/hwkan/image.png", color_img);

    ros::waitForShutdown();
}