#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/StdVector>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <livox_ros_driver/CustomMsg.h>
#include "common.h"
#include "result_verify.h"

using namespace std;
using namespace cv;


class LidarRGBfusion
{
public:
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud_RGB;
    typedef pcl::PointCloud<pcl::PointXYZI> PointCloud_I;
    typedef pcl::PointXYZRGB point;
    typedef livox_ros_driver::CustomMsg livox_msg;

    LidarRGBfusion()
    {
        voxel_sor.setLeafSize(0.01f, 0.01f, 0.01f);
        outlier_sor.setMeanK(20);
        outlier_sor.setStddevMulThresh(0.1);
    }

    void lidar_camera_registration(vector<livox_msg> msgs)
    {
        points.clear();
        for (size_t i = 0; i < msgs.size(); i++)
        {
            registration_rgb_lidar(msgs[i]);
        }
    }

    void lidar_camera_registration(vector<PointCloud_I> msgs)
    {
        points.clear();
        for (size_t i = 0; i < msgs.size(); i++)
        {
            registration_rgb_lidar(msgs[i]);
        }
    }

    void lidar_camera_registration(PointCloud_I msgs)
    {
        points.clear();
        registration_rgb_lidar(msgs);
    }

    void set_rgb_image(cv::Mat img)
    {
        color_image = img;
        depth_image = cv::Mat::zeros(color_image.size(), CV_64FC3);
    }

    void set_intrinsic_path(string path)
    {
        intrinsic_path = path;
    }

    void set_extrinsic_path(string path)
    {
        extrinsic_path = path;
    }

    void get_parameters()
    {
        getIntrinsic(intrinsic_path, intrinsic);
        getIntrinsic(intrinsic_path, distortion);
        getExtrinsic(extrinsic_path, extrinsic);
    }

    cv::Mat return_depth_image()
    {
        return depth_image;
    }

    PointCloud_RGB return_point_cloud()
    {
        return points;
    }

    void set_min_distance(double value)
    {
        min_distance = value;
    }

    void set_max_distance(double value)
    {
        max_distance = value;
    }

    void filter(){
        outlier_sor.setInputCloud(points.makeShared());
        outlier_sor.filter(points);
        // voxel_sor.setInputCloud(points.makeShared());
        // voxel_sor.filter(points);
    }

private:
    string intrinsic_path, extrinsic_path;
    vector<float> intrinsic, distortion, extrinsic;
    double min_distance = 0.3, max_distance = 3.0;
    cv::Mat color_image, depth_image;
    size_t point_size;
    PointCloud_RGB points;
    float theoryUV[2] = {0, 0};
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_sor;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_sor;

    void registration_rgb_lidar(livox_msg cloud)
    {
        for (size_t i = 0; i < cloud.points.size(); i++)
        {
            // obtain x,y,z from pointcloud msg
            double x = cloud.points[i].x;
            double y = cloud.points[i].y;
            double z = cloud.points[i].z;
            // get calibrated u,v from transformation
            getTheoreticalUV(theoryUV, intrinsic, extrinsic, x, y, z);
            int u = floor(theoryUV[0] + 0.5);
            int v = floor(theoryUV[1] + 0.5);
            // align x,y,z to depth image
            if (u > 0 && v > 0 && v < color_image.rows && u < color_image.cols)
            {
                depth_image.at<Vec3d>(v, u)[0] = x;
                depth_image.at<Vec3d>(v, u)[1] = y;
                depth_image.at<Vec3d>(v, u)[2] = z;
                // align r,g,b to point cloud
                if (x >= min_distance && x <= max_distance)
                {
                    point p;
                    p.x = x;
                    p.y = y;
                    p.z = z;
                    double r = color_image.at<Vec3b>(v, u)[2];
                    double g = color_image.at<Vec3b>(v, u)[1];
                    double b = color_image.at<Vec3b>(v, u)[0];
                    points.points.push_back(p);
                }
            }
        }
    }

    void registration_rgb_lidar(PointCloud_I cloud)
    {
        for (size_t i = 0; i < cloud.points.size(); i++)
        {
            // obtain x,y,z from pointcloud msg
            double x = cloud.points[i].x;
            double y = cloud.points[i].y;
            double z = cloud.points[i].z;
            // get calibrated u,v from transformation
            getTheoreticalUV(theoryUV, intrinsic, extrinsic, x, y, z);
            int u = floor(theoryUV[0] + 0.5);
            int v = floor(theoryUV[1] + 0.5);
            // align x,y,z to depth image
            if (u > 0 && v > 0 && v < color_image.rows && u < color_image.cols)
            {
                double prev_x = depth_image.at<Vec3d>(v, u)[0];
                if (prev_x == 0)
                {
                    prev_x = 10000;
                }
                if (x < prev_x)
                {
                    depth_image.at<Vec3d>(v, u)[0] = x;
                    depth_image.at<Vec3d>(v, u)[1] = y;
                    depth_image.at<Vec3d>(v, u)[2] = z;
                    // align r,g,b to point cloud
                    if (x >= min_distance && x <= max_distance)
                    {
                        point p;
                        p.x = x;
                        p.y = y;
                        p.z = z;
                        p.r = color_image.at<Vec3b>(v, u)[2];
                        p.g = color_image.at<Vec3b>(v, u)[1];
                        p.b = color_image.at<Vec3b>(v, u)[0];
                        points.points.push_back(p);
                    }
                }
            }
        }
    }
};