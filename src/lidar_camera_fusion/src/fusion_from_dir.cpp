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

#include <iostream>

using namespace std;

class directory{
public:
    directory(string a);
    directory(string a, vector<string> &b);
    string getInputDir();
    void setDir(string a);
    vector<string> getFileNameSet();
    void getFileName(string inputDir);

private:
    string inputDir;
    vector<string> fileNameSet;
};

directory::directory(string a)
{
    this->inputDir = a;
}

directory::directory(string a, vector<string> &b)
{
    this->inputDir = a;
    this->fileNameSet = b;
}

string directory::getInputDir()
{
    return this->inputDir;
}

void directory::setDir(string a)
{
    inputDir = a;
}

vector<string> directory::getFileNameSet()
{
    return this->fileNameSet;
}
void directory::getFileName(string inputDir)
{
    if(inputDir.empty())
    {   
        return;
    }
    struct stat statBuf;
    mode_t modes;
    lstat(inputDir.c_str(), &statBuf);
    modes = statBuf.st_mode;
    if(S_ISREG(modes))
    {
        fileNameSet.push_back(inputDir);
        return;
    }
    if(S_ISDIR(modes))
    {
        struct dirent *dir;
        DIR *dp;
        if((dp = opendir(inputDir.c_str())) == NULL)
        {
            cerr<<"open directory error";
            return;
        }
        while((dir = readdir(dp)) != NULL)
        {
            if(strcmp(".", dir->d_name) ==0 || strcmp("..", dir->d_name) == 0)
            {
                continue;
            }
            string subFileName = inputDir + "/" + dir->d_name;
            lstat(subFileName.c_str(), &statBuf);
            if(S_ISREG(statBuf.st_mode))
            {
                fileNameSet.push_back(subFileName);
            }
            if(S_ISDIR(statBuf.st_mode))
            {
                getFileName(subFileName);
            }
        }
        closedir(dp);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion_from_pcd");
    ros::NodeHandle nh;
    double min_distance, max_distance;
    std::string file_path, save_path, intrinsic_path, extrinsic_path;
    
    ros::param::get("intrinsic", intrinsic_path);
    ros::param::get("extrinsic", extrinsic_path);
    ros::param::get("min_distance", min_distance);
    ros::param::get("max_distance", max_distance);
    ros::param::get("file_path", file_path);
    ros::param::get("save_path", save_path);

    ROS_INFO_STREAM(file_path);
    directory dir(file_path);
    dir.getFileName(file_path);
    vector<string> tmp;
    tmp = dir.getFileNameSet();
    ROS_INFO_STREAM(tmp.size());
    // simple  c++ 11 new feature
    for(const auto a : tmp)
    {
        cout<<a<<endl;
    }
    return 0;
}